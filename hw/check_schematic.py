"""Schematic checks for KiCad 7 (whose CLI has no ERC):

1. Round trip: export the netlist with kicad-cli from the generated .kicad_sch and check
   that KiCad's own connectivity matches parts.py exactly, pin by pin.
2. Electrical rules (a subset of KiCad's ERC matrix): no net with a single pin, every
   power-input net has a driver or PWR_FLAG, no two power outputs shorted, no output
   pins driving each other, every input pin driven by something.
"""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(__file__))
from parts import PARTS  # noqa: E402

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stratonode")
SCH = os.path.join(D, "stratonode.kicad_sch")
XML = os.path.join(D, "stratonode.xml")


def main():
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml", "-o", XML, SCH],
                   check=True, capture_output=True)
    root = ET.parse(XML).getroot()
    kicad = {}
    types = {}
    for net in root.iter("net"):
        name = net.get("name").lstrip("/")
        for node in net.iter("node"):
            ref = node.get("ref")
            if ref.startswith("#"):
                continue
            kicad[(ref, node.get("pin"))] = name
            types[(ref, node.get("pin"))] = node.get("pintype")
    errors = []
    want = {}
    for p in PARTS:
        for pin, net in p["pins"].items():
            if net:
                want[(p["ref"], pin)] = net
    for k, net in want.items():
        got = kicad.get(k)
        if got != net:
            errors.append(f"connectivity: {k} expected {net}, KiCad has {got}")
    for k, net in kicad.items():
        if k not in want and not net.startswith("unconnected-"):
            errors.append(f"connectivity: {k} on {net} in KiCad but not in parts.py")

    nets = {}
    for k, net in kicad.items():
        if not net.startswith("unconnected-"):
            nets.setdefault(net, []).append((k, (types[k] or "").split("+")[0]))
    # PWR_FLAGs are not exported in netlists: read them from the schematic (flag pin + label at one point)
    from sexp import parse, find, find1
    tree = parse(open(SCH).read())[0]
    flag_at = {(round(float(find1(s, "at")[1]), 2), round(float(find1(s, "at")[2]), 2))
               for s in find(tree, "symbol") if find1(s, "lib_id") and find1(s, "lib_id")[1] == "power:PWR_FLAG"}
    flagged = {str(l[1]) for l in find(tree, "label")
               if (round(float(find1(l, "at")[1]), 2), round(float(find1(l, "at")[2]), 2)) in flag_at}
    for net, pins in nets.items():
        t = [x[1] for x in pins]
        if len(pins) < 2:
            errors.append(f"ERC: net {net} has one pin")
        if "power_in" in t and "power_out" not in t and net not in flagged:
            errors.append(f"ERC: power input on {net} not driven")
        if t.count("power_out") > 1:
            errors.append(f"ERC: {net} has {t.count('power_out')} power outputs")
        if t.count("output") > 1:
            errors.append(f"ERC: {net} has several outputs")
        if "input" in t and not any(x in t for x in ("output", "bidirectional", "passive", "tri_state",
                                                     "power_out", "open_collector", "power_in")) and net not in flagged:
            errors.append(f"ERC: input on {net} not driven")
    print(f"KiCad netlist: {len(nets)} nets, {sum(len(v) for v in nets.values())} connected pins; "
          f"parts.py: {len(want)} connected pins")
    if errors:
        print("\n".join(errors))
        sys.exit(1)
    print("connectivity matches parts.py exactly; ERC subset: 0 errors")


if __name__ == "__main__":
    main()
