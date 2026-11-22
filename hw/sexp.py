"""Minimal S-expression reader/writer for KiCad files."""
import re


class Q(str):
    """A quoted string token."""


_TOK = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)


def parse(text: str):
    stack, cur = [], []
    pos = 0
    n = len(text)
    while pos < n:
        m = _TOK.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise ValueError(f"bad token at {pos}: {text[pos:pos + 40]!r}")
        pos = m.end()
        if m.group(1):
            stack.append(cur)
            cur = []
        elif m.group(2):
            done = cur
            cur = stack.pop()
            cur.append(done)
        elif m.group(3) is not None:
            cur.append(Q(m.group(3).replace('\\"', '"').replace("\\\\", "\\")))
        else:
            cur.append(m.group(4))
    return cur


def dump(x, indent=0) -> str:
    if isinstance(x, list):
        if not x:
            return "()"
        simple = all(not isinstance(e, list) for e in x)
        if simple or len(dump_flat(x)) < 100:
            return dump_flat(x)
        pad = "  " * (indent + 1)
        inner = "\n".join(pad + dump(e, indent + 1) for e in x[1:])
        return "(" + dump(x[0]) + "\n" + inner + ")"
    if isinstance(x, Q):
        return '"' + x.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(x, float):
        s = f"{x:.4f}".rstrip("0").rstrip(".")
        return s if s not in ("-0", "") else "0"
    return str(x)


def dump_flat(x) -> str:
    if isinstance(x, list):
        return "(" + " ".join(dump_flat(e) for e in x) + ")"
    return dump(x)


def find(node, key):
    return [e for e in node if isinstance(e, list) and e and e[0] == key]


def find1(node, key):
    r = find(node, key)
    return r[0] if r else None
