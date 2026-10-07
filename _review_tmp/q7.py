import json, sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
devs, nets = d['devices'], d['nets']
# build refdes-pin -> (net, full line)
pinfo = {}
for name, pins in nets.items():
    for p in pins:
        tok = p.split()[0]
        pinfo[tok] = (name, p)

def dump(dev, maxpin=None):
    v = devs[dev]
    print(f"### {dev}  PARTTYPE={v.get('parttype')}  FOOTPRINT={v.get('footprint')}  Value={v.get('value')}")
    # collect pins of this device
    got = {}
    for tok,(net,line) in pinfo.items():
        rd, pin = tok.split('-',1)
        if rd == dev:
            got[pin] = (net, line)
    def keyf(p):
        try: return (0, int(p))
        except: return (1, p)
    for pin in sorted(got, key=keyf):
        net, line = got[pin]
        print(f"{dev}-{pin} -> {net}    | {line}")
    print()

for dev in ['U14','U8','U1','U2','U3','U4','U12','U7','U16','U17','U9','U10','U11','U5','U6','U15','Q1','Q7','USB1','CN1','CN2','H2','H3','SW1','SW3','SW4','ST-KEY','ST-RST','ESP-BOOT','ESP-RST','X1','X2','L1','L3','L4','SW2','I2C-1','I2C-2','I2C-3','ABZ-1','ABZ-2','ABZ-3','B00','B01','B10','B11','R36','AT103-NTC','RNTC','RVST']:
    dump(dev)
