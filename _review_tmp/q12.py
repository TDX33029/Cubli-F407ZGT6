import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
B = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_SCHEM.json', encoding='utf-8'))
pmap = {}
for name, pins in B['nets'].items():
    for p in pins:
        toks = p.split()
        pmap.setdefault(toks[0], toks[1])
for dev in ['CN1','CN2','H2','H3','I2C-1','I2C-2','I2C-3','ABZ-1','ABZ-2','ABZ-3','DRV8313-1','DRV8313-2','DRV8313-3','UNDEFINE','3V3','LED1','LED2','LED3','LED4','LED5','LED10','LED11','LED12','LED13','LED14','LED16','LED17','LED18','LED19','LED20','LED21','X1','X2','L1','L3','L4','R6','C25','C29','C31','C35','C76','C83']:
    rows = [(t, pmap[t]) for t in sorted(pmap, key=lambda s:(len(s),s)) if t.split('-',1)[0] == dev]
    if rows:
        print(f"{dev}: " + " | ".join(f"{t}->{f}" for t,f in rows))
