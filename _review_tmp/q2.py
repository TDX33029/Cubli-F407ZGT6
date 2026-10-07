import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
devs = d['devices']
# print everything that's not a pure R/C/L
import re
for des in d['device_order']:
    v = devs[des]
    dev_name = (v.get('device') or '')
    pt = (v.get('parttype') or '')
    val = (v.get('value') or '')
    if dev_name.startswith(('R_','C_','L_','R0','C0','L0')) or re.fullmatch(r'[RCL]\d+', des):
        continue
    print(f"{des:8s} | PT={pt:45s} | V={val:20s} | FP={v.get('footprint')}")
