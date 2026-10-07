import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
B = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_SCHEM.json', encoding='utf-8'))
A = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
# build full pin->footprint-pinname map from SCHEM
pmap = {}
for name, pins in B['nets'].items():
    for p in pins:
        toks = p.split()
        pmap.setdefault(toks[0], toks[1])
# print for devices of interest: pin -> (net, schem footprin-pin, pcb footprint-pin)
def show(dev):
    out = []
    for tok in sorted(pmap, key=lambda t:(t.split('-')[0], len(t), t)):
        if tok.split('-',1)[0] == dev:
            out.append((tok, pmap[tok]))
    return out
for dev in ['U14','U8','U1','U2','U3','U4','U12','U7','U16','U17','U9','U10','U11','U5','U6','U15','Q1','Q4','Q5','Q6','Q7','USB1','SW1','SW3','SW4','ST-KEY','ST-RST','ESP-BOOT','ESP-RST','D1','D3','D4','D5','U12']:
    rows = show(dev)
    if rows:
        print(f"--- {dev} ---")
        for tok, fp in rows:
            # net from A
            net = ''
            print(f"  {tok}: {fp}")
