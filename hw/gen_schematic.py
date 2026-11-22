"""Generate stratonode.kicad_sch (KiCad 7) from parts.py.

Each symbol is placed un-rotated inside its functional block; every connected pin gets a
net label at its end, unconnected pins get no-connect flags, and nets that have power-input
pins but no power-output driver get a PWR_FLAG. Library symbols are copied from the
installed KiCad libraries (derived symbols are flattened) and embedded in the file.
"""
import os
import sys
import uuid as _uuid

sys.path.insert(0, os.path.dirname(__file__))
from sexp import Q, parse, dump, find, find1  # noqa: E402
from parts import PARTS  # noqa: E402

SYMDIR = "/usr/share/kicad/symbols"
OUT = os.path.join(os.path.dirname(__file__), "stratonode", "stratonode.kicad_sch")
_ns = _uuid.UUID("6b1d6a5e-6d2a-4c1f-9a7e-000000000001")


def uid(*key):
    return Q(str(_uuid.uuid5(_ns, "/".join(map(str, key)))))


ROOT = uid("root")
_libcache = {}


def load_lib(lib):
    if lib not in _libcache:
        tree = parse(open(os.path.join(SYMDIR, lib + ".kicad_sym")).read())[0]
        _libcache[lib] = {e[1]: e for e in find(tree, "symbol")}
    return _libcache[lib]


def rename(node, old, new):
    if isinstance(node, list):
        if len(node) > 1 and node[0] == "symbol" and isinstance(node[1], str) and node[1].startswith(old + "_"):
            node[1] = Q(new + node[1][len(old):])
        for e in node:
            rename(e, old, new)


def flat_symbol(lib_id):
    """Return a deep-copied, flattened symbol named 'Lib:Name' for embedding."""
    lib, name = lib_id.split(":")
    syms = load_lib(lib)
    s = syms[name]
    ext = find1(s, "extends")
    if ext:
        parent = parse(dump(syms[ext[1]]))[0]
        rename(parent, ext[1], name)
        child_props = {p[1]: p for p in find(s, "property")}
        out = [parent[0], Q(name)]
        for e in parent[2:]:
            if isinstance(e, list) and e[0] == "property" and e[1] in child_props:
                out.append(child_props.pop(e[1]))
            else:
                out.append(e)
        out[2:2] = list(child_props.values())
        s = out
    else:
        s = parse(dump(s))[0]
    s[1] = Q(lib_id)
    return s


def pins_of(sym):
    """[(number, name, etype, x, y, angle, hidden)] for unit 1 / body style 1."""
    res = []
    for sub in find(sym, "symbol"):
        parts = str(sub[1]).rsplit("_", 2)
        unit, style = int(parts[-2]), int(parts[-1])
        if unit not in (0, 1) or style not in (0, 1):
            continue
        for p in find(sub, "pin"):
            at = find1(p, "at")
            num = find1(p, "number")[1]
            nm = find1(p, "name")[1]
            res.append((str(num), str(nm), p[1], float(at[1]), float(at[2]), int(float(at[3])), "hide" in p))
    return res


def bbox(pins):
    xs = [p[3] for p in pins] or [0]
    ys = [p[4] for p in pins] or [0]
    return min(xs), max(xs), min(ys), max(ys)


def font(size=1.27, hide=False, justify=None):
    e = ["effects", ["font", ["size", size, size]]]
    if justify:
        e.append(["justify"] + justify)
    if hide:
        e.append("hide")
    return e


def snap(v, g=2.54):
    return round(v / g) * g


def main():
    lib_symbols, used = [], set()
    items = []
    nets_pins = {}       # net -> list of (ref, pin, etype)
    pin_cache = {}

    def need(lib_id):
        if lib_id not in used:
            used.add(lib_id)
            s = flat_symbol(lib_id)
            lib_symbols.append(s)
            pin_cache[lib_id] = pins_of(s)
        return pin_cache[lib_id]

    for p in PARTS:
        pins = need(p["lib"])
        nums = {x[0] for x in pins}
        missing = set(p["pins"]) - nums
        assert not missing, (p["ref"], missing)
        for n, net in p["pins"].items():
            if net:
                et = [x[2] for x in pins if x[0] == n][0]
                nets_pins.setdefault(net, []).append((p["ref"], n, et))
        unmapped = nums - set(p["pins"])
        assert not unmapped or p["lib"] == "Mechanical:MountingHole", (p["ref"], "unmapped pins", unmapped)

    for net, lst in nets_pins.items():
        assert len(lst) >= 2, f"net {net} has a single connection: {lst}"

    # blocks laid out on an A1 sheet in a 3 x 3 grid
    blocks = sorted({p["block"] for p in PARTS})
    origin = {b: (25.4 + 270 * (i % 3), 35 + 185 * (i // 3)) for i, b in enumerate(blocks)}
    cursor = {b: [0.0, 0.0, 0.0] for b in blocks}       # x, y, row height
    BLOCK_W = 255.0

    for b in blocks:
        ox, oy = origin[b]
        items.append(["text", Q(b.split("  ", 1)[1].upper()), ["at", ox, oy - 12, 0], font(3.0, justify=["left"])])

    for p in PARTS:
        pins = need(p["lib"])
        x0, x1, y0, y1 = bbox(pins)
        label_len = max([len(n) for n in p["pins"].values() if n] + [4]) * 1.1 + 4
        w = (x1 - x0) + 2 * label_len + 6
        h = (y1 - y0) + 14
        cur = cursor[p["block"]]
        if cur[0] + w > BLOCK_W:
            cur[0], cur[1], cur[2] = 0.0, cur[1] + cur[2], 0.0
        ox, oy = origin[p["block"]]
        X = snap(ox + cur[0] + label_len - x0 + 3)
        Y = snap(oy + cur[1] + y1 + 6)
        cur[0] += w
        cur[2] = max(cur[2], h)

        sym = ["symbol", ["lib_id", Q(p["lib"])], ["at", X, Y, 0], ["unit", 1], ["in_bom", "yes"],
               ["on_board", "yes"], ["dnp", "no"], ["uuid", uid("sym", p["ref"])],
               ["property", Q("Reference"), Q(p["ref"]), ["at", X + x1 + 1.5, Y - y1 - 1.5, 0], font(justify=["left"])],
               ["property", Q("Value"), Q(p["value"]), ["at", X + x1 + 1.5, Y - y1 + 1.0, 0], font(justify=["left"])],
               ["property", Q("Footprint"), Q(p["fp"]), ["at", X, Y, 0], font(hide=True)],
               ["property", Q("Datasheet"), Q(""), ["at", X, Y, 0], font(hide=True)],
               ["property", Q("MPN"), Q(p["mpn"]), ["at", X, Y, 0], font(hide=True)]]
        for num, *_ in pins:
            sym.append(["pin", Q(num), ["uuid", uid("pin", p["ref"], num)]])
        sym.append(["instances", ["project", Q("stratonode"),
                                  ["path", Q("/" + ROOT), ["reference", Q(p["ref"])], ["unit", 1]]]])
        items.append(sym)

        seen = set()
        for num, nm, et, px, py, ang, hidden in pins:
            ex, ey = round(X + px, 3), round(Y - py, 3)
            if (ex, ey) in seen:
                continue
            seen.add((ex, ey))
            net = p["pins"].get(num)
            if net:
                la = (ang + 180) % 360
                just = {0: ["left"], 180: ["right"], 90: ["left"], 270: ["right"]}[la]
                items.append(["label", Q(net), ["at", ex, ey, la], ["fields_autoplaced"],
                              font(justify=just + ["bottom"] if la in (0, 180) else just),
                              ["uuid", uid("lab", p["ref"], num)]])
            elif et != "no_connect" or True:
                items.append(["no_connect", ["at", ex, ey], ["uuid", uid("nc", p["ref"], num)]])

    # PWR_FLAGs where a net has power inputs but no power output
    need("power:PWR_FLAG")
    fx, fy = 25.4, 570.0
    flagged = []
    for net, lst in sorted(nets_pins.items()):
        types = {t for _, _, t in lst}
        if "power_in" in types and "power_out" not in types:
            k = len(flagged)
            X, Y = snap(fx + 30 * k), snap(fy)
            items.append(["symbol", ["lib_id", Q("power:PWR_FLAG")], ["at", X, Y, 0], ["unit", 1], ["in_bom", "yes"],
                          ["on_board", "yes"], ["dnp", "no"], ["uuid", uid("flag", net)],
                          ["property", Q("Reference"), Q(f"#FLG{k + 1:02d}"), ["at", X, Y - 4, 0], font(hide=True)],
                          ["property", Q("Value"), Q("PWR_FLAG"), ["at", X, Y - 4, 0], font()],
                          ["property", Q("Footprint"), Q(""), ["at", X, Y, 0], font(hide=True)],
                          ["property", Q("Datasheet"), Q(""), ["at", X, Y, 0], font(hide=True)],
                          ["pin", Q("1"), ["uuid", uid("flagpin", net)]],
                          ["instances", ["project", Q("stratonode"),
                                         ["path", Q("/" + ROOT), ["reference", Q(f"#FLG{k + 1:02d}")], ["unit", 1]]]]])
            items.append(["label", Q(net), ["at", X, Y + 0.0, 270], ["fields_autoplaced"], font(justify=["right"]),
                          ["uuid", uid("flaglab", net)]])
            flagged.append(net)
    items.append(["text", Q("Power flags (nets fed from connectors or the buck inductor): " + ", ".join(flagged)),
                  ["at", fx, fy - 12, 0], font(2.0, justify=["left"])])

    sch = ["kicad_sch", ["version", 20230121], ["generator", "eeschema"], ["uuid", ROOT], ["paper", Q("A1")],
           ["title_block", ["title", Q("StratoNode v0.1 - stratospheric avionics power & telemetry node")],
            ["rev", Q("0.1")], ["comment", 1, Q("Generated from hw/parts.py by hw/gen_schematic.py")]],
           ["lib_symbols"] + lib_symbols] + items + [["sheet_instances", ["path", Q("/"), ["page", Q("1")]]]]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write(dump(sch) + "\n")
    print(f"wrote {OUT}: {len(PARTS)} parts, {len(nets_pins)} nets, {len(flagged)} power flags")


if __name__ == "__main__":
    main()
