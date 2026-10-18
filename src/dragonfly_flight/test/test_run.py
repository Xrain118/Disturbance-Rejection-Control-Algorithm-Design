import socket
import unittest
from unittest import mock

from dragonfly_flight import run


class RunPreflightTest(unittest.TestCase):
    @mock.patch.object(run, '_processes_using_executable', return_value=[])
    def test_px4_preflight_accepts_free_udp_port(self, unused_processes):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
        probe.close()
        run._check_px4_resources('/does/not/exist', port)

    @mock.patch.object(run, '_processes_using_executable', return_value=[])
    def test_px4_preflight_reports_busy_udp_port(self, unused_processes):
        owner = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        owner.bind(('0.0.0.0', 0))
        port = owner.getsockname()[1]
        try:
            with self.assertRaisesRegex(RuntimeError, f'UDP {port} is already in use'):
                run._check_px4_resources('/does/not/exist', port)
        finally:
            owner.close()

    @mock.patch.object(run, '_processes_using_executable', return_value=[(1234, 'px4')])
    def test_px4_preflight_reports_existing_px4(self, unused_processes):
        with self.assertRaisesRegex(RuntimeError, r'PX4 instance 0 is already running \(PID 1234\)'):
            run._check_px4_resources('/does/not/exist', 0)


if __name__ == '__main__':
    unittest.main()
