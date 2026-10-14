"""Exercise the actual corner script against a simulated serial device, no motors."""
import contextlib
import io
import json
import runpy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / 'diagnostics/serial_corner_trial.py'


class Device:
    def __init__(self, fault=None):
        self.now = 0.
        self.is_open = False
        self.in_waiting = 0
        self.commands = []
        self.pending = []
        self.fault = fault
        self.fast = False
        self.arm_at = None
        self.state = dict(App='0', BMode='1', Bal='0', EN='000', Fault='0', Trip='0',
                          Aligned='111', Cal='0' if fault == 'uncalibrated' else '1', IMU='1',
                          AxisMask='0', DirMask='0', Ref='0', VLimit='5', SLimit='180', TripDeg='20.0')
        if fault != 'legacy':
            self.state['CornerStart'] = '1'
            self.state.update(YawTune='1', YawD='3.00')

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def state_frame(self):
        return '$STATE,' + ','.join(f'{k}:{v}' for k, v in self.state.items()) + '#\n'

    def write(self, raw):
        command = raw.decode().strip()
        self.commands.append(command)
        parts = command.split()
        name = parts[0]
        if name in ('STOP', 'APP', 'BMODE', 'BAL'):
            self.state.update(EN='000', Bal='0')
            self.arm_at = None
        if name == 'APP':
            self.state.update(App=parts[1], Trip='0')
        elif name == 'BMODE':
            self.state['BMode'] = parts[1]
        elif name == 'BAXIS':
            self.state['AxisMask'] = '7'
            self.state.update({f'Axis{i}': parts[i] for i in (1, 2, 3)})
        elif name == 'BDIR':
            self.state['DirMask'] = '7'
            self.state.update({f'D{i}': parts[i] for i in (1, 2, 3)})
        elif name in ('U', 'L', 'BTRIP'):
            self.state[{'U': 'VLimit', 'L': 'SLimit', 'BTRIP': 'TripDeg'}[name]] = str(float(parts[1]))
        elif name in ('BZERO', 'BREF'):
            self.state['Ref'] = '1'
        elif name == 'BYAW':
            self.state['YawD'] = parts[1]
        elif name == 'FAST':
            self.fast = parts[1] == '7'
        elif name == 'BSTART':
            self.state.update(Bal='5', EN='000')
            self.arm_at = self.now + float(parts[1])
        elif command == 'BAL 1':
            self.state.update(Bal='3', EN='111')
        if name == 'STATE?':
            self.pending.append(self.state_frame())
        return len(raw)

    def read(self, count):
        self.now += .02
        if self.arm_at is not None and self.now >= self.arm_at:
            self.state.update(Bal='3', EN='111')
        if self.pending:
            return self.pending.pop(0).encode()
        active = self.state['Bal'] == '3'
        data = dict(Bal=self.state['Bal'], EN=self.state['EN'], Pit='23.56', Rol='46.79',
                    Gx='100' if active and self.fault == 'gyro' else '.5', Gy='-.4', Gz='.2',
                    S1='0', S2='101' if active and self.fault == 'M2_speed' else '0', S3='0')
        if active and self.fault == 'nan':
            data['S3']='nan'
        if self.fast:
            data.update(V1=data['S1'], V2=data['S2'], V3=data['S3'],
                        Wx='1.746' if active and self.fault == 'gyro' else '0', Wy='0', Wz='0')
            return ('$FAST,' + ','.join(f'{k}:{v}' for k, v in data.items()) + '#\n').encode()
        return ('$TELE,' + ','.join(f'{k}:{v}' for k, v in data.items()) + '#\n').encode()


class CornerDiagnosticTests(unittest.TestCase):
    def run_trial(self, fault=None, prepare=False, saved=False, yaw=None):
        device = Device(fault)
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            pose = folder / 'pose.json'
            pose.write_text(json.dumps({'ranges': {'Pit': {'median': 23.56}, 'Rol': {'median': 46.79}}}))
            argv = [str(SCRIPT), '--tag', 'sim', '--output-dir', directory,
                    '--expected-pose', str(pose), '--body-axes', '2', '3', '1',
                    '--directions', '-1', '-1', '-1'] + (['--prepare-only'] if prepare else [])
            if saved:
                argv += ['--reference-gravity', '-.668', '.4', '.627']
            if yaw is not None:
                argv += ['--yaw-damping', str(yaw)]
            code = 0
            with patch.object(sys, 'argv', argv), patch('serial.Serial', return_value=device), \
                    patch('time.monotonic', side_effect=lambda: device.now), \
                    patch.object(sys,'path',[str(SCRIPT.parent),*sys.path]), contextlib.redirect_stdout(io.StringIO()):
                try:
                    runpy.run_path(str(SCRIPT), run_name='__main__')
                except SystemExit as error:
                    code = error.code
            result = json.loads((folder / 'sim_summary.json').read_text())
        self.assertFalse(device.is_open)
        self.assertEqual(device.state['EN'], '000')
        self.assertGreaterEqual(device.commands.count('STOP'), 2)
        return device, result, code

    def test_prepare_never_arms(self):
        device, result, code = self.run_trial(prepare=True)
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'prepared_disabled')
        self.assertNotIn('BAL 1', device.commands)

    def test_bounded_corner_arms_all_axes_and_stops(self):
        device, result, code = self.run_trial()
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'bounded_run_completed')
        self.assertIn('BAXIS 2 3 1', device.commands)
        self.assertIn('BSTART 8', device.commands)
        self.assertIn('BCAP 2', device.commands)
        self.assertGreaterEqual(result['run_duration_s'], 3)

    def test_m2_speed_trips_and_stops_every_axis(self):
        _, result, code = self.run_trial(fault='M2_speed')
        self.assertEqual(code, 2)
        self.assertIn('wheel speed bound', result['error'])

    def test_body_rate_trips(self):
        _, result, code = self.run_trial(fault='gyro')
        self.assertEqual(code, 2)
        self.assertIn('body rate/tilt bound', result['error'])
    def test_nonfinite_feedback_trips(self):
        _, result, code=self.run_trial(fault='nan')
        self.assertEqual(code,2)
        self.assertIn('Non-finite',result['error'])

    def test_uncalibrated_device_never_arms(self):
        device, result, code = self.run_trial(fault='uncalibrated')
        self.assertEqual(code, 2)
        self.assertNotIn('BAL 1', device.commands)
        self.assertIn('calibration', result['error'])
    def test_saved_corner_reference_and_firmware_countdown(self):
        device, result, code = self.run_trial(saved=True)
        self.assertEqual(code, 0)
        self.assertTrue(any(c.startswith('BREF ') for c in device.commands))
        self.assertNotIn('BZERO', device.commands)
        self.assertIn('FAST 7', device.commands)
        self.assertIn('BSTART 8', device.commands)
    def test_legacy_uses_host_wait_without_fast(self):
        device, result, code = self.run_trial(fault='legacy')
        self.assertEqual(code, 0)
        self.assertIn('BAL 1', device.commands)
        self.assertNotIn('FAST 7', device.commands)
    def test_legacy_refuses_saved_corner_reference(self):
        device, result, code = self.run_trial(fault='legacy', saved=True)
        self.assertEqual(code, 2)
        self.assertNotIn('BAL 1', device.commands)
        self.assertIn('requires updated firmware', result['error'])
    def test_independent_yaw_is_applied_and_read_back_before_arming(self):
        device, result, code = self.run_trial(yaw=2)
        self.assertEqual(code, 0)
        self.assertIn('BYAW 2.0', device.commands)
        self.assertLess(device.commands.index('BYAW 2.0'), device.commands.index('BSTART 8'))
    def test_legacy_refuses_unavailable_yaw_setting(self):
        device, result, code = self.run_trial(fault='legacy', yaw=2)
        self.assertEqual(code, 2)
        self.assertNotIn('BAL 1', device.commands)
        self.assertIn('yaw tuning requires updated firmware', result['error'])


if __name__ == '__main__':
    unittest.main()
