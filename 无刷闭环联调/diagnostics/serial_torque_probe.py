"""Fixture-only M1 torque identification. Each pulse times out inside the MCU."""
import argparse,json,time,statistics
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('--tag',required=True);a=p.parse_args()
folder=Path(__file__).resolve().parent/'2026-10-14'
log=(folder/(a.tag+'.jsonl')).open('x',encoding='utf-8')
rows=[];frames=[];buf=bytearray();phase='connect';start=time.monotonic();ping=0;state={};guard=False
s=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.3);s.dtr=False;s.rts=False;s.port='COM9'
summary={'pulses':[],'result':'incomplete'}
def record(kind,data):
    r={'t':time.monotonic()-start,'phase':phase,'kind':kind,'data':data};rows.append(r);log.write(json.dumps(r)+'\n');log.flush()
def cmd(c):s.write((c+'\r\n').encode());record('tx',c)
def pump(duration):
    global buf,ping,state
    end=time.monotonic()+duration
    while time.monotonic()<end:
        if time.monotonic()>=ping:cmd('PING');ping=time.monotonic()+.25
        buf.extend(s.read(s.in_waiting or 1))
        while b'\n' in buf:
            line,_,buf=buf.partition(b'\n');line=line.decode('ascii','replace').strip()
            if not line:continue
            record('rx',line)
            if line.startswith('ERR') and phase!='finish' and line!='ERR SAFETY_TRIP:1':raise RuntimeError(line)
            if not (line.startswith(('$STATE,','$FAST,')) and line.endswith('#')):continue
            d=dict(x.split(':',1) for x in line.split(',',1)[1][:-1].split(','));frames.append((time.monotonic()-start,phase,d))
            if 'App' in d:state=d
            if guard and (int(d.get('Fault',0)) or int(d.get('Trip',0)) or d.get('Bal')=='4' or ('EN' in d and d['EN'][1:]!='00')):raise RuntimeError('Torque probe protection: '+line)
def wait(c,t=.1):cmd(c);pump(t)
try:
    s.open();wait('PING');wait('STOP');wait('APP 0');wait('MODE 1');wait('U 3');wait('L 90');wait('ACC 400');wait('BAXIS1 2');wait('STATE?')
    if state.get('Cal')!='1' or state.get('Aligned','000')[0]!='1':raise RuntimeError('M1 alignment/static IMU calibration required')
    wait('FAST 1')
    for level,duration in [(1,80),(1.5,80),(2,60),(2.5,50)]:
        for sign in (1,-1):
            phase=f'brake_{level}_{sign}';wait('APP 0');wait('EN1 1');wait('M1 0',.8)
            latest=next(d for t,p,d in reversed(frames) if 'V1' in d)
            if abs(float(latest['V1']))>2:raise RuntimeError('Wheel has not stopped')
            wait('EN1 0');wait('APP 1');wait('BMODE 1');phase=f'baseline_{level}_{sign}';pump(.4)
            baseline=[float(d['Rate']) for t,p,d in frames if p==phase and 'Rate' in d]
            phase=f'pulse_{level}_{sign}';guard=True;cmd(f'BPROBE {sign*level} {duration}');pump(.35);guard=False
            active=[d for t,p,d in frames if p==phase and d.get('Bal')=='7' and 'T' in d]
            if len(active)<2:raise RuntimeError('Insufficient active probe samples')
            if state.get('EN')!='000' or state.get('Bal')!='0':raise RuntimeError('MCU did not end probe/disarm')
            rates=[float(d['Rate']) for d in active]
            d={'voltage':sign*level,'duration_ms':duration,'samples':len(active),'baseline_rate':statistics.mean(baseline) if baseline else None,
               'rates':rates,'speed_peak':max(abs(float(d['V1'])) for d in active),
               'wx':[float(d['Wx']) for d in active],'wy':[float(d['Wy']) for d in active],'wz':[float(d['Wz']) for d in active]}
            summary['pulses'].append(d);print(json.dumps(d),flush=True)
    summary['result']='completed'
except BaseException as e:
    summary['result']='stopped';summary['error']=repr(e);print(repr(e),flush=True)
finally:
    guard=False;phase='finish'
    if s.is_open:
        try:
            cmd('STOP');pump(.2);wait('FAST 0');wait('U 2.5');wait('L 15');wait('STATE?')
        finally:s.close()
    summary['last_state']=state
    (folder/(a.tag+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');log.close()
