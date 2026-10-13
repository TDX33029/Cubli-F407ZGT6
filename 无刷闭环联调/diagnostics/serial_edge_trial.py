"""One supervised M1 edge attempt: wait pose, MCU countdown, bounded run, STOP on exit."""
import argparse,json,time
from pathlib import Path
import serial

p=argparse.ArgumentParser()
p.add_argument('--direction',type=int,choices=(-1,1),required=True)
p.add_argument('--tag',required=True)
p.add_argument('--run-seconds',type=float,default=5)
p.add_argument('--reuse-reference',action='store_true')
p.add_argument('--speed-bound',type=float,default=40)
p.add_argument('--wait-seconds',type=float,default=60)
p.add_argument('--wide-window',action='store_true')
p.add_argument('--update-direction',action='store_true')
a=p.parse_args()
if not (0<a.run_seconds<=15 and 20<=a.speed_bound<=60 and 15<=a.wait_seconds<=120):raise SystemExit('Held trial limits: <=15s, speed bound 20..60, wait 15..120s')
folder=Path(__file__).resolve().parent/'2026-10-14'
log=(folder/(a.tag+'.jsonl')).open('x',encoding='utf-8')
rows=[];frames=[];buf=bytearray();phase='connect';start=time.monotonic();ping=0;state={};watch=False;last_rx=0;run_started=None;result='incomplete';freq_queried=False
s=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.3);s.dtr=False;s.rts=False;s.port='COM9'
def record(kind,data):
    r={'t':time.monotonic()-start,'phase':phase,'kind':kind,'data':data};rows.append(r);log.write(json.dumps(r)+'\n');log.flush()
def cmd(c):s.write((c+'\r\n').encode());record('tx',c)
def pump(duration):
    global buf,ping,state,last_rx,run_started,result
    end=time.monotonic()+duration
    while time.monotonic()<end:
        now=time.monotonic()
        if now>=ping:cmd('PING');ping=now+.25
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            line,_,buf=buf.partition(b'\n');line=line.decode('ascii','replace').strip()
            if not line:continue
            record('rx',line)
            if line.startswith('ERR') and phase!='finish' and not (phase=='capture_target' and line.startswith('ERR BZERO')) and not (not watch and line=='ERR SAFETY_TRIP:1'):
                raise RuntimeError(line)
            if not(line.startswith(('$STATE,','$FAST,','$TELE,')) and line.endswith('#')):continue
            d=dict(item.split(':',1) for item in line.split(',',1)[1][:-1].split(','));frames.append((time.monotonic()-start,d));last_rx=now
            if 'App' in d:state=d
            if watch:
                if 'EN' in d and d['EN'][1:]!='00':raise RuntimeError('Unexpected M2/M3 enable')
                if int(d.get('Fault',0)) or int(d.get('Trip',0)):raise RuntimeError('Firmware fault/trip')
                if int(d.get('Bal',0))==4:
                    result='firmware_fall_protection';raise RuntimeError('Firmware fall protection; already disarmed')
                if int(d.get('Bal',0))==3:
                    if run_started is None:run_started=now;cmd('FREQ 1');print('BALANCING started; PG6 on',flush=True)
                    if abs(float(d.get('Tilt',0)))>8:raise RuntimeError('First trial 8-degree host tilt bound')
                    if abs(float(d.get('V1',d.get('S1',0))))>a.speed_bound:raise RuntimeError('Held trial wheel speed bound')
                    if abs(float(d.get('Rate',0)))>2:raise RuntimeError('First trial body rate bound')
        if watch and now-last_rx>.4:raise RuntimeError('Telemetry timeout')
def wait(c,t=.12):cmd(c);pump(t)
try:
    s.open();wait('PING');wait('STOP');wait('APP 0');wait('STATE?')
    if state.get('Aligned','000')[0]!='1' or state.get('Cal')!='1':raise RuntimeError('M1 alignment/static calibration required')
    if a.reuse_reference and not (state.get('Ref')=='1' and state.get('BMode')=='1' and state.get('Axis1')=='2' and (a.update_direction or state.get('D1')==str(a.direction)) and int(state.get('AxisMask',0))&1 and int(state.get('DirMask',0))&1):
        raise RuntimeError('Saved reference/configuration does not match M1 edge')
    wait('U 1.5');wait('L 60');wait('APP 1');wait('BMODE 1')
    if not a.reuse_reference:
        wait('BAXIS1 2');wait(f'BDIR1 {a.direction}')
    elif a.update_direction:wait(f'BDIR1 {a.direction}')
    wait('LQR1 20 1.2 0.02 0');wait('BU 1.5')
    if a.wide_window:wait('BCAP 10')
    phase='capture_target';pump(.5)
    if not a.reuse_reference:
        for attempt in range(10):
            wait('BZERO',.3)
            if state.get('Ref')=='1':break
    if state.get('Ref')!='1':raise RuntimeError('Target capture failed; hold chosen edge still')
    phase='wait_pose_and_countdown';wait('FAST 1');wait('FREQ 1');wait('BSTART 8',.15)
    watch=True;last_rx=time.monotonic();deadline=time.monotonic()+a.wait_seconds
    previous=None
    while time.monotonic()<deadline:
        pump(.05)
        if state.get('Bal')!=previous:
            previous=state.get('Bal');print('State',previous,'CountMs',state.get('CountMs'),'Tilt',state.get('Tilt'),flush=True)
        if run_started is not None and not freq_queried and time.monotonic()-run_started>=a.run_seconds-.3:
            cmd('FREQ?');freq_queried=True
        if run_started is not None and time.monotonic()-run_started>=a.run_seconds:
            result='bounded_run_completed';break
    else:result='pose/countdown_timeout'
except BaseException as e:
    if result=='incomplete':result='stopped_by_guard'
    record('error',repr(e));print(repr(e),flush=True)
finally:
    run_ended=time.monotonic();watch=False;phase='finish'
    if s.is_open:
        try:
            cmd('STOP');pump(.2);wait('FAST 0');wait('U 2.5');wait('L 15');wait('STATE?')
        finally:s.close()
    summary={'settings':vars(a),'result':result,'run_duration':run_ended-run_started if run_started else 0,'last_state':state,
             'last_running_states':[d for t,d in frames if d.get('Bal') in ('3','4')][-10:],
             'frequency':[r['data'] for r in rows if r['kind']=='rx' and r['data'].startswith('$FREQ,')]}
    (folder/(a.tag+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');log.close()
    print(json.dumps(summary),flush=True)
