"""Grouped BOM from parts.py -> hw/stratonode/bom.csv"""
import csv, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from parts import PARTS
groups = {}
for p in PARTS:
    k = (p["value"], p["mpn"], p["fp"].split(":")[1])
    groups.setdefault(k, []).append(p)
out = os.path.join(os.path.dirname(__file__), "stratonode", "bom.csv")
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Qty", "References", "Value", "Part / rating", "Footprint", "Why this part"])
    for (v, mpn, fp), ps in sorted(groups.items(), key=lambda kv: kv[1][0]["ref"]):
        w.writerow([len(ps), " ".join(p["ref"] for p in ps), v, mpn, fp, next((p["note"] for p in ps if p["note"]), "")])
print(f"{len(PARTS)} parts, {len(groups)} lines -> {out}")
