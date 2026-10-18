"""Wire contract shared by live acquisition, recordings, replay and tests."""
from dataclasses import dataclass
import json
import math

IDS = list(range(11, 18))
ARRAYS = ('pos', 'speed', 'current', 'load', 'voltage', 'temperature', 'status', 'error', 'tau', 'read_us')
MAX_LINE = 2048
KT = [1.0787315] * 6 + [0.81395195]
PARAMETER_VERSION = 'factory-v1'

class ProtocolError(ValueError):
    pass

def number(value):
    return type(value) in (int, float) and math.isfinite(value)

def integer(value, low, high):
    return type(value) is int and low <= value <= high

@dataclass
class Frame:
    data: dict
    raw: str
    legacy: bool = False

def parse(line, mode='v1', expected_device=''):
    try:
        if len(line.encode('utf8')) > MAX_LINE:
            raise ProtocolError('oversize')
        d = json.loads(line)
    except (ValueError, UnicodeError) as exc:
        raise ProtocolError(str(exc)) from exc
    if not isinstance(d, dict):
        raise ProtocolError('not an object')
    if type(d.get('T')) is not int:
        raise ProtocolError('message type must be integer')
    if d.get('T') == 6061:
        return None
    if mode == 'legacy':
        if d.get('T') != 1051:
            raise ProtocolError('not legacy telemetry')
        if not all(number(d.get(k)) for k in ('b', 's', 'e', 't', 'r')):
            raise ProtocolError('invalid legacy angles')
        if expected_device:
            raise ProtocolError('legacy has no verifiable device identity')
        return Frame(d, line, True)
    if mode != 'v1' or d.get('T') != 1052 or type(d.get('ver')) is not int or d['ver'] != 1:
        raise ProtocolError('unsupported telemetry version/type')
    if not isinstance(d.get('dev'), str) or not 1 <= len(d['dev']) <= 32:
        raise ProtocolError('invalid device identity')
    if expected_device and d['dev'] != expected_device:
        raise ProtocolError('device identity mismatch')
    for key in ('boot', 'seq', 'overrun', 'dropped', 'scan_us'):
        if not integer(d.get(key), 0, 2**32-1):
            raise ProtocolError('invalid '+key)
    if not integer(d.get('t_us'), 0, 2**64-1) or not integer(d.get('rate'), 1, 50):
        raise ProtocolError('invalid time/rate')
    if d.get('ids') != IDS or any(type(x) is not int for x in d['ids']):
        raise ProtocolError('invalid servo identity/order')
    if not isinstance(d.get('ok'), list) or len(d['ok']) != 7 or any(type(x) is not bool for x in d['ok']):
        raise ProtocolError('invalid validity array')
    for key in ARRAYS:
        if not isinstance(d.get(key), list) or len(d[key]) != 7:
            raise ProtocolError('invalid array '+key)
        for i, value in enumerate(d[key]):
            if not d['ok'][i]:
                if value is not None:
                    raise ProtocolError('invalid servo must use null')
            elif key == 'tau':
                if not number(value):
                    raise ProtocolError('invalid torque')
            elif key == 'read_us':
                if not integer(value, 0, d['t_us']):
                    raise ProtocolError('invalid read timestamp')
            elif key in ('voltage', 'temperature', 'status', 'error'):
                if not integer(value, 0, 255):
                    raise ProtocolError('invalid byte field')
            elif not integer(value, -32767, 32767):
                raise ProtocolError('invalid raw value')
    for key in ('method', 'cal', 'params'):
        if not isinstance(d.get(key), str) or not 1 <= len(d[key]) <= 64:
            raise ProtocolError('missing estimate metadata')
    return Frame(d, line)

class LineDecoder:
    """Discard an overlong line through its delimiter; never accept its tail."""
    def __init__(self, limit=MAX_LINE):
        self.limit = limit
        self.buffer = bytearray()
        self.discarding = False
        self.errors = 0

    def feed(self, data):
        lines = []
        for byte in data:
            if byte == 10:
                if not self.discarding:
                    try:
                        line = self.buffer.rstrip(b'\r').decode('utf8')
                        if line:
                            lines.append(line)
                    except UnicodeError:
                        self.errors += 1
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                self.buffer.append(byte)
                if len(self.buffer) > self.limit:
                    self.errors += 1
                    self.buffer.clear()
                    self.discarding = True
        return lines

def joint_positions(frame, signs=(1,)*5, offsets=(0,)*5):
    if frame.legacy:
        return [(i, frame.data[k]*signs[i]+offsets[i]) for i, k in enumerate(('b','s','e','t','r'))]
    d = frame.data
    indices = (0, 1, 3, 4, 5)
    directions = (-1, 1, 1, 1, -1)
    zeros = (math.pi, -math.pi, -math.pi/2, -math.pi, math.pi)
    return [(j, (directions[j]*d['pos'][i]*2*math.pi/4096 + zeros[j])*signs[j]+offsets[j])
            for j, i in enumerate(indices) if d['ok'][i]]

def joint_torques(frame, signs=(-1,1,-1,1,1,-1,1)):
    if frame.legacy:
        return [None]*5
    d = frame.data
    physical = [d['tau'][i]*signs[i] if d['ok'][i] else None for i in range(7)]
    shoulder = physical[1]+physical[2] if physical[1] is not None and physical[2] is not None else None
    return [physical[0], shoulder, physical[3], physical[4], physical[5]]

def configure_command(operation, rate=50):
    if type(operation) is not int or operation not in (0, 1, 2) or (operation and not integer(rate, 1, 50)):
        raise ValueError('operation must be 0..2; rate must be 1..50')
    d = {'T':606}
    if operation:
        d.update(enable=operation==2, rate=rate)
    return (json.dumps(d, separators=(',',':'))+'\n').encode()

class Quality:
    def __init__(self):
        self.boot = None
        self.seq = None
        self.device_time = None
        self.last_receive = None
        self.gaps = 0
        self.resets = 0
        self.frames = 0
        self.first_receive = None
        self.last_device = None

    def accept(self, frame, monotonic):
        discontinuity = False
        if not frame.legacy:
            d = frame.data
            if self.boot is not None and d['boot'] == self.boot:
                delta = (d['seq']-self.seq) & 0xffffffff
                if delta == 0 or delta >= 2**31 or d['t_us'] <= self.device_time:
                    raise ProtocolError('duplicate or out-of-order sample')
                self.gaps += delta-1
                discontinuity = delta != 1
            elif self.boot is not None:
                self.resets += 1
                discontinuity = True
            self.boot, self.seq, self.device_time = d['boot'], d['seq'], d['t_us']
            self.last_device = d['dev']
        self.last_receive = monotonic
        if self.first_receive is None:
            self.first_receive = monotonic
        self.frames += 1
        return discontinuity

    def stale(self, now, timeout):
        return self.last_receive is None or now-self.last_receive > timeout

class TorqueFilter:
    def __init__(self, alpha=0.0):
        if not 0 <= alpha <= 1:
            raise ValueError('filter_alpha must be in [0,1]')
        self.alpha = alpha
        self.previous = [None]*5

    def update(self, values, reset=False):
        if reset:
            self.previous = [None]*5
        out = []
        for i, x in enumerate(values):
            p = self.previous[i]
            y = x if x is None or p is None or not self.alpha else self.alpha*x+(1-self.alpha)*p
            self.previous[i] = y
            out.append(y)
        return out
