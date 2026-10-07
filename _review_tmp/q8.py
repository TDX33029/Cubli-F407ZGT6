import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
devs = d['devices']
for des in ['R42','C64','C65','C66','R28','R6','R44','R36','AT103-NTC','RNTC','RVST','R59','R16','R18','R17','R31','R32','R33','R37','R60','B00','B01','B10','B11']:
    v = devs.get(des)
    if v: print(des, '| parttype:', v.get('parttype'), '| value:', v.get('value'), '| fp:', v.get('footprint'))
    else: print(des, 'NOT FOUND')
# component value helper for nets
def val(des):
    v = devs.get(des, {})
    return v.get('value') or v.get('parttype') or '?'
# show values for all resistors on specific nets
nets = d['nets']
for netname in ['$2N265','$2N280','$2N281','$2N277','$2N276','$2N275','$2N263','$2N264','$2N266','$2N267','$2N269','$2N270','$2N252','$2N251','$2N247','$2N243','$2N241','$2N233','$2N224','$2N220','$2N230']:
    comps = []
    for p in nets[netname]:
        tok = p.split()[0]
        rd = tok.rsplit('-',1)[0]
        comps.append(f"{tok}({val(rd)})")
    print(f"{netname}: {', '.join(comps)}")
