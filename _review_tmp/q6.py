import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
nets = d['nets']
# per-device pin->net map
for dev in ['USB1','CN1','CN2','H2','H3','SW1','SW3','SW4','ST-KEY','ST-RST','ESP-BOOT','ESP-RST','X1','X2','U15','L3','Q7','SW2']:
    m = {}
    for name, pins in nets.items():
        for p in pins:
            toks = p.split()
            if toks[0].startswith(dev+'-'):
                m.setdefault(toks[0].split('-',1)[1], []).append(name)
    print(f"--- {dev} ---")
    for k in sorted(m):
        print(f"  {dev}-{k}: {','.join(m[k])}")
print()
print('total nets:', len(nets))
print('net names:', ' | '.join(d['net_order']))
