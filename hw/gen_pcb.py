"""Build the StratoNode PCB from the KiCad netlist exported from the schematic (KiCad 7 Python).

    python3 gen_schematic.py && python3 check_schematic.py   # schematic + netlist round-trip check
    python3 gen_pcb.py place     # board outline, footprints, nets, placement (no overlaps)
    python3 gen_pcb.py route     # GND / +3V3 inner planes, Specctra DSN export, prints the Freerouting command
    java -jar freerouting-1.9.0.jar -de stratonode/stratonode.dsn -do stratonode/stratonode.ses -mp 40
    python3 gen_pcb.py import    # import routes, pour outer GND, DRC report, Gerbers/drill/pos, SVG renders
"""
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "stratonode")
PCB = os.path.join(D, "stratonode.kicad_pcb")
PRO = os.path.join(D, "stratonode.kicad_pro")
XML = os.path.join(D, "stratonode.xml")
FPDIR = "/usr/share/kicad/footprints"

W, H = 80.0, 56.0            # board outline, mm
OX, OY = 100.0, 100.0        # board origin on the sheet
MM = pcbnew.FromMM

HOLES = {"H1": (3.5, 3.5), "H2": (76.5, 3.5), "H3": (3.5, 52.5), "H4": (76.5, 52.5), "H5": (44.0, 3.5), "H6": (44.0, 52.5)}

# fixed placements: ref -> (x, y, rotation) ; edge connectors: ref -> (x, y, facing)
EDGE = {"J1": (8.0, 30.0, "left"), "J5": (76.5, 13.5, "right"), "J6": (76.5, 26.5, "right"), "J7": (76.5, 39.5, "right"),
        "J3": (56.0, 52.5, "down"), "J4": (67.0, 52.5, "down"), "J10": (55.0, 3.6, "up")}
FIXED = {"J8": (13.0, 5.0, 0), "J9": (26.0, 5.0, 0),
         "C2": (12.0, 47.5, 0), "C3": (23.5, 47.5, 0), "C4": (35.0, 47.5, 0),
         "Q1": (26.0, 36.0, 0), "U1": (24.0, 28.0, 0), "U4": (24.0, 16.0, 0),
         "L1": (40.0, 38.0, 0), "U2": (34.0, 30.0, 0), "U3": (47.0, 30.0, 0),
         "U5": (58.0, 25.0, 45), "J2": (66.0, 12.0, 0), "U11": (60.0, 43.0, 0),
         "U6": (69.0, 31.5, 90), "U7": (69.0, 41.0, 90), "U8": (36.0, 14.0, 0),
         "U9": (46.0, 13.0, 0), "U10": (47.0, 22.0, 0)}
ANCHOR = {"D1": "J1", "R1": "Q1", "C1": "U1", "C5": "U4", "R2": "U5", "R3": "U5", "C6": "U5",
          **{r: "U2" for r in ("R4", "R5", "R6", "C7", "C8", "C9", "C10", "C11", "R7", "R8")},
          **{r: "L1" for r in ("C12", "C13", "C14")}, "C15": "U3", "C16": "U3",
          "C17": "U6", "R9": "U6", "C19": "U6", "C18": "U7", "R10": "U7", "C20": "U7",
          "R11": "U8", "C21": "U8", "Q2": "J8", "R12": "J8", "R13": "J8", "D2": "J8",
          "Q3": "J9", "R14": "J9", "R15": "J9", "D3": "J9",
          **{r: "U5" for r in ("C22", "C23", "C24", "C25", "C26", "FB1", "C27", "C28", "Y1", "C31", "R16")},
          "C29": "Y1", "C30": "Y1", "D4": "J2", "D5": "J2", "R17": "J2", "R18": "J2",
          "C32": "U9", "C33": "U10", "R19": "U10", "R20": "U10", "R21": "U4", "R22": "U8", "R23": "J10", "C34": "J10",
          "C35": "U11", "C36": "U11", "R24": "U11", "JP1": "J3", "R25": "J3", "R26": "J3", "C37": "J3",
          "R27": "J5", "R28": "J5"}

NETCLASSES = [
    dict(name="Default", clearance=0.15, track_width=0.2, via_diameter=0.6, via_drill=0.3),
    dict(name="Power", clearance=0.2, track_width=0.8, via_diameter=0.8, via_drill=0.4),
    dict(name="Rail", clearance=0.2, track_width=0.5, via_diameter=0.6, via_drill=0.3),
]
PATTERNS = [("Power", p) for p in ("VIN", "VIN_S", "VBAT", "VBAT_HTR", "HTR1_LO", "HTR2_LO", "BUCK_SW", "+5V")] + \
           [("Rail", p) for p in ("+5V_RADIO", "+5V_PAYLOAD", "+3V3", "+3V3A")]


def netlist():
    root = ET.parse(XML).getroot()
    comps = {c.get("ref"): dict(value=c.findtext("value"), fp=c.findtext("footprint"))
             for c in root.iter("comp")}
    pins = {}
    for net in root.iter("net"):
        name = net.get("name").lstrip("/")
        if name.startswith("unconnected-"):
            continue
        for n in net.iter("node"):
            pins[(n.get("ref"), n.get("pin"))] = name
    return comps, pins


def bbox_mm(fp):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False, False)
    return (pcbnew.ToMM(bb.GetLeft()) - OX, pcbnew.ToMM(bb.GetTop()) - OY,
            pcbnew.ToMM(bb.GetRight()) - OX, pcbnew.ToMM(bb.GetBottom()) - OY)


def place_at(fp, x, y, rot):
    fp.SetOrientationDegrees(rot)
    fp.SetPosition(pcbnew.VECTOR2I(MM(OX + x), MM(OY + y)))


def overlaps(a, b, gap=0.25):
    return not (a[2] + gap <= b[0] or b[2] + gap <= a[0] or a[3] + gap <= b[1] or b[3] + gap <= a[1])


def inside(b, inset=0.4):
    return b[0] >= inset and b[1] >= inset and b[2] <= W - inset and b[3] <= H - inset


def stage_place():
    comps, pins = netlist()
    board = pcbnew.NewBoard(PCB)
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.m_CopperEdgeClearance = MM(0.3)
    ds.m_TrackMinWidth = MM(0.15)
    ds.m_MinClearance = MM(0.15)
    ds.m_ViasMinSize = MM(0.5)
    ds.m_MinThroughDrill = MM(0.3)
    edge = pcbnew.PCB_SHAPE(board)
    edge.SetShape(pcbnew.SHAPE_T_RECT)
    edge.SetStart(pcbnew.VECTOR2I(MM(OX), MM(OY)))
    edge.SetEnd(pcbnew.VECTOR2I(MM(OX + W), MM(OY + H)))
    edge.SetLayer(pcbnew.Edge_Cuts)
    edge.SetWidth(MM(0.1))
    board.Add(edge)

    nets = {}
    for name in sorted(set(pins.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni

    fps = {}
    for ref, c in comps.items():
        lib, name = c["fp"].split(":")
        fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + ".pretty"), name)
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.Value().SetVisible(False)
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        fp.Reference().SetTextThickness(MM(0.12))
        board.Add(fp)
        for pad in fp.Pads():
            net = pins.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(nets[net])
        fps[ref] = fp

    placed = {}

    def commit(ref):
        placed[ref] = bbox_mm(fps[ref])

    for ref, (x, y) in HOLES.items():
        place_at(fps[ref], x, y, 0)
        commit(ref)
    facing_vec = {"left": (-1, 0), "right": (1, 0), "up": (0, -1), "down": (0, 1)}
    for ref, (x, y, face) in EDGE.items():
        best = None
        for rot in (0, 90, 180, 270):
            place_at(fps[ref], 0, 0, rot)
            b = bbox_mm(fps[ref])
            sig = [p for p in fps[ref].Pads() if p.GetNumber() not in ("", "MP")]
            px = sum(pcbnew.ToMM(p.GetPosition().x) - OX for p in sig) / len(sig)
            py = sum(pcbnew.ToMM(p.GetPosition().y) - OY for p in sig) / len(sig)
            cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
            vx, vy = facing_vec[face]
            score = vx * (cx - px) + vy * (cy - py)     # mating face: from the signal pads towards the body
            if best is None or score > best[0]:
                best = (score, rot)
        rot = best[1]
        place_at(fps[ref], 0, 0, rot)
        b = bbox_mm(fps[ref])
        # snap the courtyard's outer side to the board edge
        dx = {"left": 0.4 - b[0], "right": W - 0.4 - b[2]}.get(face, x)
        dy = {"up": 0.4 - b[1], "down": H - 0.4 - b[3]}.get(face, y)
        if face in ("left", "right"):
            dy = y - (b[1] + b[3]) / 2
        else:
            dx = x - (b[0] + b[2]) / 2
        place_at(fps[ref], dx, dy, rot)
        commit(ref)
    for ref, (x, y, rot) in FIXED.items():
        place_at(fps[ref], x, y, rot)
        commit(ref)

    errs = []
    keys = list(placed)
    for i, a in enumerate(keys):
        if not inside(placed[a], 0.0):
            errs.append(f"{a} outside board {placed[a]}")
        for b in keys[i + 1:]:
            if overlaps(placed[a], placed[b], 0.0):
                errs.append(f"fixed overlap {a}/{b}")
    if errs:
        print("\n".join(errs))

    # spiral-pack passives around their anchor, larger parts first
    order = sorted((r for r in comps if r not in placed),
                   key=lambda r: -((lambda b: (b[2] - b[0]) * (b[3] - b[1]))(bbox_mm(fps[r]))))
    for ref in order:
        anc = ANCHOR.get(ref)
        ab = placed[anc]
        ax, ay = (ab[0] + ab[2]) / 2, (ab[1] + ab[3]) / 2
        done = False
        for r in [k * 0.5 for k in range(0, 60)]:
            steps = max(8, int(2 * math.pi * r / 0.5))
            for s in range(steps):
                t = 2 * math.pi * s / steps
                x, y = ax + r * math.cos(t), ay + r * math.sin(t)
                for rot in (0, 90):
                    place_at(fps[ref], x, y, rot)
                    b = bbox_mm(fps[ref])
                    if inside(b) and not any(overlaps(b, o) for o in placed.values()):
                        placed[ref] = b
                        done = True
                        break
                if done:
                    break
            if done:
                break
        if not done:
            errs.append(f"could not place {ref}")
            print("could not place", ref)

    t = pcbnew.PCB_TEXT(board)
    t.SetText("StratoNode v0.1")
    t.SetLayer(pcbnew.B_SilkS)
    t.SetPosition(pcbnew.VECTOR2I(MM(OX + W / 2), MM(OY + H / 2)))
    t.SetTextSize(pcbnew.VECTOR2I(MM(2), MM(2)))
    t.SetMirrored(True)
    board.Add(t)
    board.Save(PCB)
    write_project()
    print(f"placed {len(placed)} footprints, {len(errs)} problems")
    return not errs


def write_project():
    pro = {"meta": {"filename": "stratonode.kicad_pro", "version": 1},
           "net_settings": {"meta": {"version": 3},
                            "classes": [dict(c, bus_width=12, diff_pair_gap=0.25, diff_pair_via_gap=0.25,
                                             diff_pair_width=0.2, line_style=0, microvia_diameter=0.3,
                                             microvia_drill=0.1, pcb_color="rgba(0, 0, 0, 0.000)",
                                             schematic_color="rgba(0, 0, 0, 0.000)", wire_width=6) for c in NETCLASSES],
                            "netclass_patterns": [{"netclass": c, "pattern": p} for c, p in PATTERNS]},
           "board": {"design_settings": {"rules": {"min_clearance": 0.15, "min_track_width": 0.15,
                                                   "min_via_diameter": 0.5, "min_through_hole_diameter": 0.3,
                                                   "min_copper_edge_clearance": 0.3}}}}
    with open(PRO, "w") as f:
        json.dump(pro, f, indent=2)


def add_zone(board, layer, netname, inset=0.5):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(board.FindNet(netname))
    z.SetLocalClearance(MM(0.25))
    z.SetMinThickness(MM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(MM(0.25))
    z.SetThermalReliefSpokeWidth(MM(0.35))
    o = z.Outline()
    o.NewOutline()
    for x, y in ((inset, inset), (W - inset, inset), (W - inset, H - inset), (inset, H - inset)):
        o.Append(MM(OX + x), MM(OY + y))
    board.Add(z)
    return z


def stage_route(passes=40):
    board = pcbnew.LoadBoard(PCB)
    for z in list(board.Zones()):
        board.Remove(z)
    add_zone(board, pcbnew.In1_Cu, "GND")
    add_zone(board, pcbnew.In2_Cu, "+3V3")
    board.Save(PCB)
    board = pcbnew.LoadBoard(PCB)
    dsn, ses = os.path.join(D, "stratonode.dsn"), os.path.join(D, "stratonode.ses")
    pcbnew.ExportSpecctraDSN(board, dsn)
    # inner layers are GND / +3V3 planes: let the router drop vias onto them, never route on them
    import re
    txt = open(dsn).read()
    open(dsn, "w").write(re.sub(r"(\(layer In[12]\.Cu\s*\(type )signal", r"\1power", txt))
    if not os.path.exists(ses) or os.path.getmtime(ses) < os.path.getmtime(dsn):
        print("Now autoroute (Freerouting 1.9, https://github.com/freerouting/freerouting/releases):")
        print(f"  java -jar freerouting-1.9.0.jar -de {dsn} -do {ses} -mp {passes}")
        print("then run:  python3 gen_pcb.py import")
        return


def stage_finish():
    board = pcbnew.LoadBoard(PCB)
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        add_zone(board, layer, "GND", inset=0.5)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    board = pcbnew.LoadBoard(PCB)
    rpt = os.path.join(D, "drc.rpt")
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = open(rpt).read()
    print("\n".join(l for l in txt.splitlines() if l.startswith("**")))
    unrouted = board.GetConnectivity().GetUnconnectedCount(True)
    print("unrouted connections:", unrouted)
    tracks = [t for t in board.GetTracks()]
    vias = sum(1 for t in tracks if t.GetClass() == "PCB_VIA")
    print(f"tracks {len(tracks) - vias}, vias {vias}")


def stage_fab():
    out = os.path.join(D, "fab")
    os.makedirs(out, exist_ok=True)
    layers = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
    subprocess.run(["kicad-cli", "pcb", "export", "gerbers", "--layers", layers, "-o", out + "/", PCB], check=True,
                   capture_output=True)
    subprocess.run(["kicad-cli", "pcb", "export", "drill", "-o", out + "/", PCB], check=True, capture_output=True)
    subprocess.run(["kicad-cli", "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "front",
                    "-o", os.path.join(out, "stratonode-top-pos.csv"), PCB], check=True, capture_output=True)
    for name, ly in (("top", "F.Cu,F.Silkscreen,F.Mask,Edge.Cuts"), ("bottom", "B.Cu,B.Silkscreen,Edge.Cuts"),
                     ("in1_gnd", "In1.Cu,Edge.Cuts"), ("in2_3v3", "In2.Cu,Edge.Cuts"),
                     ("assembly", "F.Fab,F.Silkscreen,Edge.Cuts")):
        svg = os.path.join(D, f"render_{name}.svg")
        args = ["kicad-cli", "pcb", "export", "svg", "--layers", ly, "--page-size-mode", "2", "--exclude-drawing-sheet",
                "-o", svg, PCB]
        if name == "bottom":
            args.insert(-1, "--mirror")
        subprocess.run(args, check=True, capture_output=True)
    print("fab outputs in", out)


if __name__ == "__main__":
    st = sys.argv[1] if len(sys.argv) > 1 else "all"
    if st in ("place", "all"):
        stage_place()
    if st in ("route", "all"):
        stage_route()
    if st == "import":
        b = pcbnew.LoadBoard(PCB)
        pcbnew.ImportSpecctraSES(b, os.path.join(D, "stratonode.ses"))
        b.Save(PCB)
        st = "finish+fab"
    if st == "finish+fab":
        stage_finish()
        stage_fab()
    if st in ("finish", "all"):
        stage_finish()
    if st in ("fab", "all"):
        stage_fab()
