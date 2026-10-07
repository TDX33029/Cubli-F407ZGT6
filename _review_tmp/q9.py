import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
A = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
B = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_SCHEM.json', encoding='utf-8'))
da, db = A['devices'], B['devices']
print("device sets equal:", set(da)==set(db))
print("devices only in PCB-netlist:", sorted(set(da)-set(db)))
print("devices only in Schem-netlist:", sorted(set(db)-set(da)))
diffdev = []
for d in sorted(set(da)&set(db)):
    if (da[d].get('footprint'), da[d].get('parttype')) != (db[d].get('footprint'), db[d].get('parttype')):
        diffdev.append((d, da[d].get('parttype'), db[d].get('parttype')))
print("device footprint/parttype differences:", len(diffdev))
for x in diffdev[:20]: print("  ", x)
na, nb = A['nets'], B['nets']
print("net name sets equal:", set(na)==set(nb), len(na), len(nb))
onlyA = sorted(set(na)-set(nb)); onlyB = sorted(set(nb)-set(na))
print("nets only in PCB-netlist:", onlyA)
print("nets only in Schem-netlist:", onlyB)
conn_diff = []
for n in sorted(set(na)&set(nb)):
    pa = sorted(p.split()[0] for p in na[n])
    pb = sorted(p.split()[0] for p in nb[n])
    if pa != pb:
        conn_diff.append((n, sorted(set(pa)-set(pb)), sorted(set(pb)-set(pa))))
print("nets with different pin memberships:", len(conn_diff))
for n, x, y in conn_diff[:40]:
    print(f"  {n}: only-PCB={x} only-Schem={y}")
# check pin-name (2nd token) differences on same net
pn_diff = []
for n in sorted(set(na)&set(nb)):
    ma = {p.split()[0]: p.split()[1] for p in na[n]}
    mb = {p.split()[0]: p.split()[1] for p in nb[n]}
    for tok in set(ma)&set(mb):
        if ma[tok] != mb[tok]:
            pn_diff.append((n, tok, ma[tok], mb[tok]))
print("pin footprint-name differences on same nets:", len(pn_diff))
for x in pn_diff[:20]: print("  ", x)
