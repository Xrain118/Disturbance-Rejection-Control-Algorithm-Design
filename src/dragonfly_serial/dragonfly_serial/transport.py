"""No ROS dependencies. Independent ports, bounded queues, timestamped JSONL."""
import json
import os
from pathlib import Path
import queue
import threading
import time
import serial
from .protocol import LineDecoder, configure_command

class Recorder:
    def __init__(self, path=''):
        self.lock = threading.Lock()
        self.file = None
        self.error = None
        if path:
            p = Path(path)
            p.parent.mkdir(parents=True, exist_ok=True)
            self.file = p.open('x', encoding='utf8')
    def write(self, event):
        if self.file:
            with self.lock:
                if self.error is not None:
                    return
                try:
                    self.file.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
                    self.file.flush()
                except OSError as exc:
                    self.error=str(exc)  # Keep acquisition alive; diagnostics expose recording failure.
    def close(self):
        if self.file:
            try:
                self.file.close()
            except OSError as exc:
                self.error=str(exc)

def replay_events(path):
    with open(path, encoding='utf8') as f:
        previous = None
        for line in f:
            event = json.loads(line)
            if event.get('side') not in ('left','right') or not isinstance(event.get('mono_ns'), int):
                raise ValueError('invalid recording event')
            # Writers can arrive in a different order across two threads; replay clamps negative gaps.
            yield event
            previous = event['mono_ns']

def validate_ports(left, right):
    if not left or not right:
        raise ValueError('both explicit port paths are required')
    if os.path.realpath(left) == os.path.realpath(right):
        raise ValueError('left and right resolve to the same device')
    try:
        if os.path.samefile(left, right):
            raise ValueError('left and right are the same device')
    except FileNotFoundError:
        pass

class PortWorker:
    def __init__(self, side, port, baud, events, recorder, reconnect=2.0):
        self.side, self.port, self.baud = side, port, baud
        self.events, self.recorder = events, recorder
        self.reconnect = reconnect
        self.stop = threading.Event()
        self.commands = queue.Queue(maxsize=4)
        self.thread = threading.Thread(target=self.run, name=side+'-serial', daemon=True)
        self.local_dropped = 0

    def emit(self, kind, **values):
        event = dict(side=self.side, kind=kind, mono_ns=time.monotonic_ns(), wall_ns=time.time_ns(), **values)
        self.recorder.write(event)
        try:
            self.events.put_nowait(event)
        except queue.Full:
            self.local_dropped += 1
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            try:
                self.events.put_nowait(event)
            except queue.Full:
                self.local_dropped += 1

    def configure(self, operation, rate):
        self.commands.put_nowait(configure_command(operation, rate))

    def run(self):
        while not self.stop.is_set():
            conn = None
            try:
                conn = serial.Serial(port=None, baudrate=self.baud, timeout=.05, write_timeout=.05,
                                     rtscts=False, dsrdtr=False, exclusive=True)
                conn.dtr = False
                conn.rts = False
                conn.port = self.port
                conn.open()
                conn.reset_input_buffer()
                decoder = LineDecoder()
                self.emit('connected', port=self.port, baud=self.baud)
                while not self.stop.is_set():
                    try:
                        cmd = self.commands.get_nowait()
                    except queue.Empty:
                        cmd = None
                    if cmd is not None:
                        if conn.write(cmd) != len(cmd):
                            raise serial.SerialTimeoutException('partial configuration write')
                        self.emit('command', raw=cmd.decode().strip())
                    previous_errors = decoder.errors
                    for line in decoder.feed(conn.read(min(max(conn.in_waiting,1),4096))):
                        self.emit('line', raw=line)
                    if decoder.errors != previous_errors:
                        self.emit('framing_error', count=decoder.errors-previous_errors)
            except (serial.SerialException, OSError, ValueError) as exc:
                self.emit('disconnected', error=str(exc))
            finally:
                if conn is not None:
                    conn.close()
                # Never replay an old configuration request automatically after reconnect.
                while not self.commands.empty():
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
            self.stop.wait(self.reconnect)

    def start(self):
        self.thread.start()
    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
