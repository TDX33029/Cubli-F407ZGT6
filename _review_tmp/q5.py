import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
nets = d['nets']
# remaining nets after IO23
seen = False
for name in d['net_order']:
    if name == 'IO23':
        seen = True
    if seen:
        pins = nets[name]
        refs = [p.split()[0] for p in pins]
        print(f"{name} ({len(pins)}): {' '.join(refs)}")
print()
# U14 pins present
present = set()
pin_net = {}
for name, pins in nets.items():
    for p in pins:
        tok = p.split()[0]
        if tok.startswith('U14-'):
            pn = int(tok.split('-')[1])
            present.add(pn)
            pin_net.setdefault(pn, []).append(name)
missing = [i for i in range(1,145) if i not in present]
print('U14 pins in nets:', len(present))
print('U14 pins NOT in any net (NC):', missing)
print()
for dev, mx in [('U1',29),('U2',29),('U3',29),('U4',16),('U8',38),('U12',26),('U16',14),('U17',26),('U7',6),('USB1',20),('CN1',6),('CN2',8),('H2',8),('H3',4),('SW1',6),('SW3',4),('SW4',6)]:
    pres = set()
    for name, pins in nets.items():
        for p in pins:
            tok = p.split()[0]
            if tok.startswith(dev+'-'):
                pres.add(int(tok.split('-')[1]))
    miss = [i for i in range(1,mx+1) if i not in pres]
    print(f"{dev}: pins present {sorted(pres)}")
    print(f"   max {mx}, missing: {miss}")
