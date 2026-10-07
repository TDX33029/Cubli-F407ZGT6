import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
nets = d['nets']
for name in d['net_order']:
    pins = nets[name]
    # condense: refdes-pin only
    refs = []
    for p in pins:
        tok = p.split()[0]
        refs.append(tok)
    print(f"{name} ({len(pins)}): {' '.join(refs)}")
