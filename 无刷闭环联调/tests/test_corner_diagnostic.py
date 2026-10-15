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
AUTO_SCRIPT = Path(__file__).resolve().parents[1] / 'diagnostics/serial_corner_auto.py'


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
        self.probe_until = None
        self.probe_voltage = 0
        self.had_active = False
        self.state = dict(App='0', BMode='1', Bal='0', EN='000', Fault='0', Trip='0',
                          Aligned='111', Cal='0' if fault == 'uncalibrated' else '1', IMU='1',
                          AxisMask='0', DirMask='0', Ref='0', VLimit='5', SLimit='180', TripDeg='20.0')
        if fault != 'legacy':
            self.state['CornerStart'] = '1'
            self.state.update(YawTune='1', YawD='3.00', TrimTune='1', BTrim='2.0', CapDeg='2.0')

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def state_frame(self):
        state = dict(self.state)
        if self.had_active and self.fault == 'disarm_state_stale' and state['Bal'] == '0':
            state['EN'] = '111'
        return '$STATE,' + ','.join(f'{k}:{v}' for k, v in state.items()) + '#\n'

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
        elif name == 'BTRIM':
            if self.fault != 'trim_not_applied':
                self.state['BTrim'] = parts[1]
        elif name == 'BCAP':
            if self.fault != 'capture_not_applied':
                self.state['CapDeg'] = parts[1]
        elif name == 'FAST':
            self.fast = parts[1] == '7'
        elif name == 'BSTART':
            self.state.update(Bal='5', EN='000')
            self.arm_at = self.now + float(parts[1])
        elif command == 'BAL 1':
            self.state.update(Bal='3', EN='111')
        elif name == 'BPROBE' and self.fault != 'missing_probe':
            self.probe_voltage = float(parts[1])
            self.probe_until = self.now + int(parts[2])/1000
            self.state.update(Bal='7', EN='010' if self.fault == 'probe_wrong_enable' else '100')
        if name == 'STATE?':
            self.pending.append(self.state_frame())
        return len(raw)

    def read(self, count):
        self.now += .02
        if self.arm_at is not None and self.now >= self.arm_at:
            self.state.update(Bal='3', EN='111')
        if self.probe_until is not None and self.now >= self.probe_until:
            self.state.update(Bal='0', EN='000')
            self.probe_until = None
        if self.pending:
            return self.pending.pop(0).encode()
        active = self.state['Bal'] == '3'
        self.had_active |= active
        data = dict(Bal=self.state['Bal'], EN=self.state['EN'], Pit='23.56', Rol='46.79',
                    Gx='100' if active and self.fault == 'gyro' else '.5', Gy='-.4', Gz='.2',
                    S1='0', S2='101' if active and self.fault == 'M2_speed' else
                    ('400' if active and self.fault == 'wheel_guard' else '0'), S3='0')
        if self.fault == 'off_target_before_arm' and not active and (
                self.arm_at is None or self.now < self.arm_at - 1):
            data.update(Pit='0', Rol='90', Gx='120')
        if active and self.fault == 'nan':
            data['S3']='nan'
        if self.fast:
            data.update(V1=data['S1'], V2=data['S2'], V3=data['S3'],
                        Wx='1.746' if active and self.fault == 'gyro' else '0', Wy='0', Wz='0')
            if self.state['BMode'] != '4' and self.fault in ('probe_edge', 'missing_probe', 'probe_wrong_enable'):
                data.pop('Pit'); data.pop('Rol')
                if self.state['Bal'] == '7':
                    data.update(U1=str(self.probe_voltage), V1=str(self.probe_voltage*5),
                                Wy=str(self.probe_voltage/10))
                else:
                    for key in ('Wx', 'Wy', 'Wz'):
                        data.pop(key)
            return ('$FAST,' + ','.join(f'{k}:{v}' for k, v in data.items()) + '#\n').encode()
        return ('$TELE,' + ','.join(f'{k}:{v}' for k, v in data.items()) + '#\n').encode()


class CornerDiagnosticTests(unittest.TestCase):
    def run_trial(self, fault=None, prepare=False, saved=False, yaw=None, probe=None, live=False):
        if probe is not None:
            saved = True
            fault = fault or 'probe_edge'
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
            if probe is not None:
                argv += ['--probe-motor','1','--probe-voltage',str(probe),'--probe-ms','80',
                         '--kp','0','--kd','0','--kw','0','0','0','--yaw-damping','0',
                         '--voltage','3','--wheel-bound','60','--seconds','1']
            if live:
                argv += ['--live-state', str(folder/'live.json')]
            code = 0
            with patch.object(sys, 'argv', argv), patch('serial.Serial', return_value=device), \
                    patch('time.monotonic', side_effect=lambda: device.now), \
                    patch.object(sys,'path',[str(SCRIPT.parent),*sys.path]), contextlib.redirect_stdout(io.StringIO()):
                try:
                    runpy.run_path(str(SCRIPT), run_name='__main__')
                except SystemExit as error:
                    code = error.code
            result = json.loads((folder / 'sim_summary.json').read_text())
            if live:
                result['_published_live_state'] = json.loads((folder/'live.json').read_text())
        self.assertFalse(device.is_open)
        self.assertEqual(device.state['EN'], '000')
        self.assertGreaterEqual(device.commands.count('STOP'), 2)
        return device, result, code

    def test_prepare_never_arms(self):
        device, result, code = self.run_trial(prepare=True)
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'prepared_disabled')
        self.assertNotIn('BAL 1', device.commands)
    def test_live_display_distinguishes_timer_and_rate_protection(self):
        _, timed, code = self.run_trial(live=True)
        self.assertEqual(code, 0)
        self.assertEqual(timed['_published_live_state']['outcome_text'], '定时结束（3秒）')
        self.assertIsNone(timed['_published_live_state']['remaining_s'])
        _, protected, code = self.run_trial(fault='gyro', live=True)
        self.assertEqual(code, 2)
        self.assertEqual(protected['_published_live_state']['outcome_text'], '角速度保护停机')
    def test_missing_disarm_confirmation_does_not_display_normal_timer_end(self):
        _, result, code = self.run_trial(fault='disarm_state_stale', live=True)
        self.assertEqual(code, 2)
        self.assertIn('final_disarm_error', result)
        self.assertEqual(result['_published_live_state']['outcome_text'], '停机未确认')

    def test_bounded_corner_arms_all_axes_and_stops(self):
        device, result, code = self.run_trial()
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'bounded_run_completed')
        self.assertIn('BAXIS 2 3 1', device.commands)
        self.assertIn('BSTART 2', device.commands)
        self.assertIn('BCAP 0.5', device.commands)
        self.assertGreaterEqual(result['run_duration_s'], 3)

    def test_finite_trial_clears_previous_experimental_trim(self):
        device, _, code = self.run_trial()
        self.assertEqual(code, 0)
        self.assertIn('BTRIM 0', device.commands)
        self.assertLess(device.commands.index('BTRIM 0'), device.commands.index('BSTART 2'))

    def test_finite_trial_refuses_trim_not_disabled(self):
        device, result, code = self.run_trial(fault='trim_not_applied')
        self.assertEqual(code, 2)
        self.assertIn('Tilt trim disabled', result['error'])
        self.assertNotIn('BSTART 2', device.commands)

    def test_finite_trial_requires_half_degree_readback(self):
        device, result, code = self.run_trial(fault='capture_not_applied')
        self.assertEqual(code, 2)
        self.assertIn('Corner configuration', result['error'])
        self.assertNotIn('BSTART 2', device.commands)

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
        self.assertIn('BSTART 2', device.commands)
    def test_saved_reference_waits_for_entry_without_hand_capture_gate(self):
        device, result, code = self.run_trial(fault='off_target_before_arm', saved=True)
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'bounded_run_completed')
        self.assertIn('BSTART 2', device.commands)
        self.assertNotIn('BZERO', device.commands)
    def test_timed_probe_waits_for_release_and_disarms_before_pulse(self):
        device, result, code = self.run_trial(probe=3)
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'probe_completed')
        self.assertGreaterEqual(len(result['probe']['samples']), 3)
        i = device.commands.index('BPROBE 3 80')
        self.assertIn('BAL 0', device.commands[:i])
        self.assertLess(device.commands.index('BSTART 2'), i)
        self.assertTrue(all(float(d['Wy']) > 0 for d in result['probe']['samples']))
    def test_negative_timed_probe_records_full_gyro_in_edge_mode(self):
        _, result, code = self.run_trial(probe=-3)
        self.assertEqual(code, 0)
        self.assertTrue(all(float(d['Wy']) < 0 for d in result['probe']['samples']))
    def test_missing_probe_response_is_not_direction_evidence(self):
        _, result, code = self.run_trial(fault='missing_probe', probe=3)
        self.assertEqual(code, 2)
        self.assertIn('Insufficient timed probe', result['error'])
    def test_probe_rejects_unexpected_enabled_motor(self):
        _, result, code = self.run_trial(fault='probe_wrong_enable', probe=3)
        self.assertEqual(code, 2)
        self.assertIn('Partial three-axis enable', result['error'])
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
        self.assertLess(device.commands.index('BYAW 2.0'), device.commands.index('BSTART 2'))
    def test_legacy_refuses_unavailable_yaw_setting(self):
        device, result, code = self.run_trial(fault='legacy', yaw=2)
        self.assertEqual(code, 2)
        self.assertNotIn('BAL 1', device.commands)
        self.assertIn('yaw tuning requires updated firmware', result['error'])


class AutoDevice(Device):
    """Corner device with BAUTO: entry, balancing segments, angle retry and hard faults."""

    def __init__(self, fault=None):
        super().__init__(fault)
        self.auto_at = None
        self.cycle_start = None
        self.segment = 0

    def write(self, raw):
        command = raw.decode().strip()
        parts = command.split()
        if parts[0] in ('STOP', 'APP', 'BMODE', 'BAL'):
            self.cycle_start = None
            self.auto_at = None
            self.state['Auto'] = '0'
        if parts[0] == 'BAUTO':
            self.commands.append(command)
            if parts[1] == '1':
                self.state.update(Auto='1', Bal='6', EN='000', BReason='0')
                self.auto_at = None if self.fault == 'no_entry' else self.now + .5
            else:
                self.state.update(Auto='0', Bal='0', EN='000')
                self.auto_at = None
            return len(raw)
        if parts[0] == 'BFULL':
            self.state['FullSpeed'] = str(float(parts[1]))
        return super().write(raw)

    def read(self, count):
        self.now += .02
        if 'Auto' in self.state and self.state['Auto'] == '1':
            self.state.setdefault('FullSpeed', '250.0')
            self.state.setdefault('Wheel', '0')
            self.state.setdefault('Rate', '0.001')
            if self.auto_at is not None and self.now >= self.auto_at:
                self.auto_at = None
                self.cycle_start = self.now
            if self.cycle_start is not None:
                elapsed = self.now - self.cycle_start
                cycle = elapsed % 3.0
                if cycle < .3:
                    self.state.update(Bal='6', EN='000')
                elif cycle < .8:
                    self.state.update(Bal='5', EN='000')
                else:
                    if self.state['Bal'] != '3':
                        self.segment += 1
                    self.state.update(Bal='3', EN='111')
                    if cycle > 1.8 and self.segment >= 1 and self.fault != 'keep_balancing':
                        if self.fault == 'fall_protect':
                            self.state.update(Bal='4', EN='000', BReason='5')
                        else:
                            self.state.update(Bal='6', EN='000', BReason='1', RetryMs='500')
                if cycle >= 2.7:
                    self.state.update(BReason='0', RetryMs='0')
        return super().read(count)


class CornerAutoTests(unittest.TestCase):
    def run_auto(self, fault=None, prepare=False, minutes=None, wait=30):
        device = AutoDevice(fault)
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            argv = [str(AUTO_SCRIPT), '--tag', 'sim', '--output-dir', directory,
                    '--body-axes', '2', '3', '1', '--directions', '-1', '-1', '-1',
                    '--wait-seconds', str(wait)] + (['--prepare-only'] if prepare else [])
            if minutes is not None:
                argv += ['--duration-minutes', str(minutes)]
            code = 0
            with patch.object(sys, 'argv', argv), patch('serial.Serial', return_value=device), \
                    patch('time.monotonic', side_effect=lambda: device.now), \
                    patch.object(sys, 'path', [str(AUTO_SCRIPT.parent), *sys.path]), \
                    contextlib.redirect_stdout(io.StringIO()):
                try:
                    runpy.run_path(str(AUTO_SCRIPT), run_name='__main__')
                except SystemExit as error:
                    code = error.code
            result = json.loads((folder / 'sim_summary.json').read_text())
        self.assertFalse(device.is_open)
        self.assertEqual(device.state['EN'], '000')
        self.assertGreaterEqual(device.commands.count('STOP'), 2)
        return device, result, code

    def test_prepare_never_arms_auto(self):
        device, result, code = self.run_auto(prepare=True)
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'prepared_disabled')
        self.assertFalse(any(c.startswith('BAUTO 1') for c in device.commands))

    def test_disabling_trim_must_be_confirmed_before_arming(self):
        device, result, code = self.run_auto(fault='trim_not_applied', prepare=True)
        self.assertEqual(code, 2)
        self.assertIn('BTRIM 0.0', device.commands)
        self.assertIn('configuration', result['error'])
        self.assertFalse(any(c.startswith('BAUTO 1') for c in device.commands))

    def test_auto_requires_half_degree_readback(self):
        device, result, code = self.run_auto(fault='capture_not_applied', prepare=True)
        self.assertEqual(code, 2)
        self.assertIn('configuration', result['error'])
        self.assertFalse(any(c.startswith('BAUTO 1') for c in device.commands))

    def test_auto_arm_config_then_balancing_attempts(self):
        device, result, code = self.run_auto(minutes=.2)
        self.assertEqual(code, 0)
        self.assertEqual(result['result'], 'timed_run_completed')
        self.assertIn('BAUTO 1 2', device.commands)
        self.assertIn('L 330', device.commands)
        self.assertIn('BFULL 250', device.commands)
        for command in ('BREF ' + ' '.join(f'{v:.9f}' for v in
                        [-0.7028287890572913, 0.3977869310324257, 0.5897433770480731]),
                        'LQR1 80 5 -0.03 0.0', 'LQR2 80 5 -0.03 0.0', 'LQR3 80 5 -0.03 0.0'):
            self.assertIn(command, device.commands)
        self.assertGreaterEqual(result['auto_attempts'], 1)
        self.assertGreaterEqual(result['balance_seconds_total'], 0.9)
        self.assertIn(result['attempts'][0]['end_reason'], (0, 1))
        self.assertLessEqual(result['attempts'][0]['max_tilt'], 28)

    def test_angle_retry_reenters_without_manual_restart(self):
        device, result, code = self.run_auto(minutes=.3)
        self.assertEqual(code, 0)
        self.assertGreaterEqual(result['auto_attempts'], 2)
        self.assertEqual(device.commands.count('BAUTO 1 2'), 1)

    def test_hard_fault_exits_automatic_mode(self):
        _, result, code = self.run_auto(fault='fall_protect', minutes=.5)
        self.assertEqual(code, 2)
        self.assertIn('protection', result['error'])

    def test_wheel_backstop_above_firmware_overspeed(self):
        _, result, code = self.run_auto(fault='wheel_guard', minutes=.5)
        self.assertEqual(code, 2)
        self.assertIn('wheel speed backstop', result['error'])

    def test_entry_timeout_when_pose_never_ready(self):
        _, result, code = self.run_auto(fault='no_entry', wait=30)
        self.assertEqual(code, 2)
        self.assertIn('entry not achieved', result['error'])

    def test_uncalibrated_device_never_arms_auto(self):
        device, result, code = self.run_auto(fault='uncalibrated')
        self.assertEqual(code, 2)
        self.assertFalse(any(c.startswith('BAUTO 1') for c in device.commands))
        self.assertIn('calibration', result['error'])


if __name__ == '__main__':
    unittest.main()
