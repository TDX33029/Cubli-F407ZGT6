import re, json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def parse(path):
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        txt = f.read()
    lines = txt.splitlines()
    devices = {}
    order = []
    i = 0
    n = len(lines)
    # parse device blocks: lines starting with '[' ... ']'
    while i < n:
        line = lines[i]
        if line.strip() == '[':
            # collect until ']'
            j = i+1
            block = []
            while j < n and lines[j].strip() != ']':
                block.append(lines[j])
                j += 1
            # block[0]=DESIGNATOR, block[1]=designator value, block[2]=FOOTPRINT, block[3]=footprint, block[4]=PARTTYPE, block[5]=parttype
            dev = {}
            if len(block) >= 6:
                dev['designator'] = block[1].strip()
                dev['footprint'] = block[3].strip()
                dev['parttype'] = block[5].strip()
            # scan for Value / Device keys
            k = 0
            while k < len(block)-1:
                if block[k].strip() == 'Value' and 'value' not in dev:
                    dev['value'] = block[k+1].strip()
                if block[k].strip() == 'Device' and 'device' not in dev:
                    dev['device'] = block[k+1].strip()
                k += 1
            d = dev.get('designator','')
            if d:
                devices[d] = dev
                order.append(d)
            i = j+1
        else:
            i += 1
    # parse nets: '(' ... ')' ; first line = net name; subsequent lines "refdes-pin footprint-pin type"
    nets = {}
    netorder = []
    i = 0
    while i < n:
        line = lines[i]
        if line.strip() == '(':
            j = i+1
            netname = lines[j].strip()
            j += 1
            pins = []
            while j < n and lines[j].strip() != ')':
                s = lines[j].strip()
                if s:
                    pins.append(s)
                j += 1
            nets[netname] = pins
            netorder.append(netname)
            i = j+1
        else:
            i += 1
    return devices, order, nets, netorder

for tag, path in [('MAIN', r'D:/Document/tempPrj/Cubli-F407ZGT6/Netlist_MB_2026-10-07.net'),
                  ('SCHEM', r'D:/Document/tempPrj/Cubli-F407ZGT6/无刷闭环联调/Netlist_MB_Schematic_2026-10-07.net')]:
    devices, dorder, nets, norder = parse(path)
    out = {'devices': devices, 'device_order': dorder, 'nets': nets, 'net_order': norder}
    with open(rf'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_{tag}.json', 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(tag, 'devices:', len(devices), 'nets:', len(nets))
