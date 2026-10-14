"""Supervised, bounded three-wheel corner trial; STOP on every exit.

Uses MCU countdown and full corner FAST when CornerStart=1 is reported.
Older firmware uses standard TELE and an eight-second host wait without PG5.
Axes/signs are the independently identified installation, not encoder signs.
"""
import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


def gravity(data):
    pitch, roll = math.radians(float(data['Pit'])), math.radians(float(data['Rol']))
    return [-math.sin(roll) * math.cos(pitch), math.sin(pitch), math.cos(roll) * math.cos(pitch)]


def tilt(left, right):
    dot = sum(a * b for a, b in zip(left, right))
    return math.degrees(math.acos(max(-1, min(1, dot))))


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='COM7')
parser.add_argument('--tag', required=True)
parser.add_argument('--seconds', type=float, default=3)
parser.add_argument('--voltage', type=float, default=3.5)
parser.add_argument('--wheel-bound', type=float, default=100)
parser.add_argument('--kp', type=float, default=80)
parser.add_argument('--kd', type=float, default=5)
parser.add_argument('--kw', type=float, nargs=3, default=[-.045, -.045, -.030])
parser.add_argument('--body-axes', type=int, nargs=3, choices=(1, 2, 3), required=True)
parser.add_argument('--directions', type=int, nargs=3, choices=(-1, 1), required=True)
parser.add_argument('--expected-pose', type=Path, required=True, help='Disabled passive pose summary JSON')
parser.add_argument('--reference-gravity', type=float, nargs=3, help='Saved normalized corner target; requires CornerStart firmware')
parser.add_argument('--prepare-only', action='store_true', help='Capture/configure, then STOP without arming')
parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent / '2026-10-14')
args = parser.parse_args()
if not (0 < args.seconds <= 5 and 0 < args.voltage <= 6 and 30 <= args.wheel_bound <= 150
        and 0 <= args.kp <= 120 and 0 <= args.kd <= 10 and all(abs(v) <= .1 for v in args.kw)
        and sorted(args.body_axes) == [1, 2, 3]):
    parser.error('Corner diagnostic bounds exceeded')
expected = json.loads(args.expected_pose.read_text(encoding='utf-8'))
expected_g = gravity({key: expected['ranges'][key]['median'] for key in ('Pit', 'Rol')})
if min(abs(v) for v in expected_g) < .25:
    parser.error('Expected pose is too close to a face/edge; capture a corner pose first')
if args.reference_gravity is not None:
    norm = math.sqrt(sum(v*v for v in args.reference_gravity))
    if not all(math.isfinite(v) for v in args.reference_gravity) or not .9 <= norm <= 1.1:
        parser.error('Saved reference must be finite and normalized')
    args.reference_gravity = [v / norm for v in args.reference_gravity]
    if min(abs(v) for v in args.reference_gravity) < .25:
        parser.error('Saved corner reference too close to an edge/face')

folder = args.output_dir
folder.mkdir(parents=True, exist_ok=True)
log = (folder / (args.tag + '.jsonl')).open('x', encoding='utf-8')
summary = {'utc_started': datetime.now(timezone.utc).isoformat(),
           'settings': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
           'result': 'incomplete', 'reference': None, 'hands_off_verified': False,
           'cable_free_verified': False, 'active_samples': 0}
link = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.3)
link.dtr = False
link.rts = False
link.port = args.port
start = time.monotonic()
phase = 'connect'
buffer = bytearray()
next_ping = 0
state = {}
revision = 0
tele = {}
recent = []
active = []
messages = []
reference = None
watch = False
run_started = None
last_tele = 0


def record(kind, data):
    log.write(json.dumps({'t': time.monotonic() - start, 'phase': phase,
                          'kind': kind, 'data': data}, ensure_ascii=False) + '\n')
    log.flush()


def send(command):
    link.write((command + '\r\n').encode('ascii'))
    record('tx', command)


def pump(duration):
    global buffer, next_ping, state, revision, tele, last_tele, run_started
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_ping:
            send('PING')
            next_ping = now + .25
        buffer.extend(link.read(link.in_waiting or 1))
        while b'\n' in buffer:
            line, _, buffer = buffer.partition(b'\n')
            line = line.decode('ascii', 'replace').strip()
            if not line:
                continue
            record('rx', line)
            messages.append(line)
            if phase != 'finish' and line.startswith('ERR') and line != 'ERR SAFETY_TRIP:1':
                raise RuntimeError(line)
            if not line.startswith(('$STATE,', '$TELE,', '$FAST,')) or not line.endswith('#'):
                continue
            data = dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(','))
            if line.startswith('$STATE,'):
                state = data
                revision += 1
            elif line.startswith('$TELE,'):
                tele = data
                last_tele = now
                recent.append((now, data))
                while recent and now - recent[0][0] > 1:
                    recent.pop(0)
            else:
                if not all(k in data for k in ('V1','V2','V3','Pit','Rol','Wx','Wy','Wz','Bal')):
                    raise RuntimeError('Incomplete corner FAST attitude')
                last_tele = now
                for i, key in enumerate(('Wx','Wy','Wz'), 1):
                    data[f'S{i}'] = data[f'V{i}']
                    data[('Gx','Gy','Gz')[i-1]] = str(float(data[key]) * 180 / math.pi)
            if watch:
                if int(data.get('Fault', 0)) or int(data.get('Trip', 0)) or data.get('Bal') == '4':
                    raise RuntimeError('Firmware protection: ' + line)
                if data.get('EN', '000') not in ('000', '111'):
                    raise RuntimeError('Partial three-axis enable: ' + line)
                if line.startswith(('$TELE,', '$FAST,')):
                    if max(abs(float(data[f'S{i}'])) for i in (1, 2, 3)) > args.wheel_bound:
                        summary['guard_frame'] = data
                        raise RuntimeError('Corner wheel speed bound')
                    rate = math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180
                    if rate > 1.5 or tilt(gravity(data), reference) > 10:
                        raise RuntimeError('Corner body rate/tilt bound')
                    if data.get('Bal') == '3':
                        if run_started is None:
                            run_started = now
                            send('FREQ 7')
                            print('BALANCING: PG6 on, release now', flush=True)
                        active.append(data)
        if len(buffer) > 65536:
            raise RuntimeError('Serial overflow')
        if watch and now - last_tele > .35:
            raise RuntimeError('Corner attitude telemetry timeout')


def command(text, seconds=.12):
    send(text)
    pump(seconds)


def confirm(predicate, label, timeout=1):
    previous = revision
    send('STATE?')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pump(.02)
        if revision > previous and predicate(state):
            return
    raise RuntimeError(label + ': fresh STATE confirmation failed')


def still_corner():
    samples = [d for t, d in recent if time.monotonic() - t <= .5]
    if len(samples) < 5 or time.monotonic() - last_tele > .15:
        return False
    for data in samples:
        g = gravity(data)
        rate = math.sqrt(sum(float(data[k]) ** 2 for k in ('Gx', 'Gy', 'Gz'))) * math.pi / 180
        if (data.get('EN') != '000' or tilt(g, expected_g) > 6 or min(abs(v) for v in g) < .25
                or rate > .1 or max(abs(float(data[f'S{i}'])) for i in (1, 2, 3)) > 2):
            return False
    return True


try:
    link.open()
    command('PING')
    command('STOP')
    command('FAST 0')
    command('TELE 1')
    command('APP 0')
    confirm(lambda d: d.get('Aligned') == '111' and d.get('Cal') == '1'
            and d.get('IMU') == '1' and d.get('EN') == '000' and d.get('Fault') == '0',
            'Three alignments/static IMU calibration')
    for text in ('APP 1', 'BMODE 4', 'BAXIS ' + ' '.join(map(str, args.body_axes)),
                 'BDIR ' + ' '.join(map(str, args.directions)), f'U {args.voltage}',
                 f'L {args.wheel_bound + 20}', f'BU {args.voltage}', 'BTRIP 20', 'FREQ 7'):
        command(text)
    for i, kw in enumerate(args.kw, 1):
        command(f'LQR{i} {args.kp} {args.kd} {kw} 0')
    confirm(lambda d: d.get('BMode') == '4' and d.get('AxisMask') == '7' and d.get('DirMask') == '7'
            and all(d.get(f'Axis{i}') == str(args.body_axes[i-1])
                    and d.get(f'D{i}') == str(args.directions[i-1]) for i in (1, 2, 3))
            and abs(float(d.get('VLimit', 0)) - args.voltage) < .11
            and abs(float(d.get('SLimit', 0)) - args.wheel_bound - 20) < .11
            and d.get('TripDeg') == '20.0' and d.get('EN') == '000', 'Corner configuration')
    phase = 'wait_still_corner'
    print('Waiting for stationary supported corner; motors disabled', flush=True)
    deadline = time.monotonic() + 60
    while not still_corner():
        if time.monotonic() >= deadline:
            raise RuntimeError('Corner pose not stationary within 60s')
        pump(.05)
    phase = 'capture_reference'
    firmware_countdown = state.get('CornerStart') == '1'
    if args.reference_gravity is not None:
        if not firmware_countdown:
            raise RuntimeError('Saved corner reference requires updated firmware')
        command('BREF ' + ' '.join(f'{v:.9f}' for v in args.reference_gravity), .2)
        reference = args.reference_gravity
    else:
        command('BZERO', .2)
        reference = gravity(tele)
    confirm(lambda d: d.get('Ref') == '1' and d.get('BMode') == '4'
            and d.get('EN') == '000', 'Corner BZERO')
    summary['reference'] = {'gravity': reference,
                            'pitch_deg': math.degrees(math.asin(reference[1])),
                            'roll_deg': math.degrees(math.atan2(-reference[0],reference[2])),
                            'neutral_verified': False}
    if args.prepare_only:
        summary['result'] = 'prepared_disabled'
    else:
        if firmware_countdown:
            phase = 'mcu_countdown'
            command('FAST 7')
            watch = True
            command('BSTART 8')
            print('MCU wait pose/countdown; PG5 flashes, hold until PG6', flush=True)
            deadline = time.monotonic() + 60
        else:
            phase = 'host_countdown'
            print('Host wait 8s; PG5 does not blink in legacy corner firmware; hold until PG6', flush=True)
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                pump(.05)
                if (time.monotonic() - last_tele > .2 or tilt(gravity(tele), reference) > 3
                        or tele.get('EN') != '000'):
                    raise RuntimeError('Corner moved during host countdown; not armed')
            phase = 'corner_run'
            watch = True
            command('BAL 1', .05)
            deadline = time.monotonic() + 8
        while run_started is None or time.monotonic() - run_started < args.seconds:
            if run_started is None and time.monotonic() >= deadline:
                raise RuntimeError('Corner takeover not achieved')
            pump(.01)
        summary['result'] = 'bounded_run_completed'
        summary['run_duration_s'] = time.monotonic() - run_started
except BaseException as error:
    summary['result'] = 'stopped'
    summary['error'] = repr(error)
    if run_started is not None:
        summary['run_duration_s'] = time.monotonic() - run_started
    record('error', repr(error))
    print(repr(error), flush=True)
finally:
    watch = False
    phase = 'finish'
    if link.is_open:
        try:
            command('STOP', .2)
            command('FAST 0')
            command('U 2.5')
            command('L 15')
            confirm(lambda d: d.get('EN') == '000' and d.get('Bal') == '0', 'Final disarm')
            command('FREQ?', .15)
        except BaseException as error:
            summary['result'] = 'stopped'
            summary['final_disarm_error'] = repr(error)
            record('error', repr(error))
        finally:
            link.close()
    summary['last_state'] = state
    summary['active_samples'] = len(active)
    if active:
        summary['tilt_deg_max'] = max(tilt(gravity(d), reference) for d in active)
        summary['wheel_ranges'] = {f'M{i}': [min(float(d[f'S{i}']) for d in active),
                                            max(float(d[f'S{i}']) for d in active)] for i in (1, 2, 3)}
    summary['frequency'] = [line for line in messages if line.startswith('$FREQ,')]
    (folder / (args.tag + '_summary.json')).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    log.close()
    print(json.dumps(summary), flush=True)
if summary['result'] not in ('bounded_run_completed', 'prepared_disabled'):
    raise SystemExit(2)
