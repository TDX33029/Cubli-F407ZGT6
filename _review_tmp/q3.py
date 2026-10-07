import json, sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
devs = d['devices']
# group RC/L by value
rc = {}
for des in d['device_order']:
    v = devs[des]
    dev_name = (v.get('device') or '')
    val = (v.get('value') or '')
    if re.fullmatch(r'[RCL]\d+', des) and (dev_name.startswith(('R_','C_','L_','R0','C0','L0')) or val.startswith(('1','2','3','4','5','6','7','8','9','0','10K','100'))):
        key = (dev_name.split('_')[0] if dev_name else '?', val)
        rc.setdefault(key, []).append(des)
print("=== RC/L groups ===")
for k in sorted(rc, key=lambda x:(x[0],x[1])):
    lst = rc[k]
    print(f"{k[0]} {k[1]:12s} x{len(lst):3d} : {' '.join(lst)}")
print()
print("=== device names unique ===")
names = {}
for des,v in devs.items():
    names.setdefault(v.get('device'), []).append(des)
for k in sorted(names, key=str):
    print(k, '->', len(names[k]), names[k][:12], '...' if len(names[k])>12 else '')
