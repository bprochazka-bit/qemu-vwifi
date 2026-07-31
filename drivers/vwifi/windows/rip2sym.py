#!/usr/bin/env python3
"""
Resolve a guest RIP to a vwifi.sys function, on the Linux host.

When the Windows guest dies without a bugcheck -- a triple fault, or a
hard lock that ends in a reset -- there is no dump and no stop screen.
The only surviving record of where the CPU was is what QEMU logged on
the host, and that is an absolute virtual address. This turns it back
into a function name:

    RVA = RIP - <image base>

The image base is printed by DriverEntry ("vwifi: image base ffff..."),
and the RVA is looked up in vwifi.map, which the build now emits beside
vwifi.sys. Nothing here needs Windows tooling or a .pdb.

Usage:
    ./rip2sym.py --map x64/Debug/vwifi.map --base 0xfffff8021a340000 \\
                 0xfffff8021a34c1f3

    # several at once, e.g. every RIP from a QEMU cpu_reset dump
    ./rip2sym.py --map vwifi.map --base 0x... 0xRIP1 0xRIP2 0xRIP3

    # or pipe QEMU's log in and let it find the RIPs itself
    grep -o 'RIP=[0-9a-f]*' qemu.log | ./rip2sym.py --map vwifi.map \\
                 --base 0x... --stdin

If a RIP is outside the image the script says so rather than inventing
a symbol -- an address below the base or past the image size belongs to
somebody else's module, which is itself the answer.
"""

import argparse
import re
import sys

# MSVC map lines look like:
#   0001:00000f40  VwifiHwStart      fffff80000001f40 f  hardware.obj
# section:offset, name, absolute-address-at-preferred-base, flags, object
MAP_LINE = re.compile(
    r"^\s*([0-9A-Fa-f]{4}):([0-9A-Fa-f]{8,16})\s+(\S+)\s+([0-9A-Fa-f]+)\s+"
    r"(\S*)\s*(\S*)\s*$"
)

PREFERRED_BASE = re.compile(r"^\s*Preferred load address is\s+([0-9A-Fa-f]+)",
                            re.IGNORECASE)


def parse_map(path):
    """Return (sorted [(rva, name, obj)], preferred_base or None)."""
    syms = []
    preferred = None

    with open(path, "r", errors="replace") as fh:
        for line in fh:
            m = PREFERRED_BASE.match(line)
            if m:
                preferred = int(m.group(1), 16)
                continue
            m = MAP_LINE.match(line)
            if not m:
                continue
            name = m.group(3)
            addr = int(m.group(4), 16)
            obj = m.group(6) or m.group(5)
            # The absolute column is relative to the preferred base; if
            # the map has not declared one yet, fall back to
            # section:offset, which is close enough to order by and is
            # corrected below once the preferred base is known.
            syms.append([addr, name, obj])

    if preferred is not None:
        for s in syms:
            s[0] -= preferred

    syms.sort(key=lambda s: s[0])
    return syms, preferred


def demangle(name):
    """MSVC decorates C++ symbols; C ones are clean. Strip the leading
    underscore MSVC adds and leave anything else alone -- a half-hearted
    guess at a C++ name is worse than the mangled truth."""
    if name.startswith("?"):
        return name          # C++, left as-is on purpose
    return name.lstrip("_") if name.startswith("_") else name


def resolve(syms, rva):
    """Nearest symbol at or below rva."""
    lo, hi = 0, len(syms) - 1
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        if syms[mid][0] <= rva:
            best = syms[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def main():
    ap = argparse.ArgumentParser(
        description="Resolve a guest RIP to a vwifi.sys symbol.")
    ap.add_argument("--map", required=True,
                    help="vwifi.map from the build output")
    ap.add_argument("--base", required=True,
                    help="image base as logged by DriverEntry, e.g. 0xfffff802...")
    ap.add_argument("--size", default=None,
                    help="image size as logged by DriverEntry, e.g. 0x1e000. "
                         "Used only to say when a RIP is out of range.")
    ap.add_argument("--stdin", action="store_true",
                    help="also read addresses from stdin (any hex found)")
    ap.add_argument("rips", nargs="*", help="one or more RIP values")
    args = ap.parse_args()

    base = int(args.base, 0)
    size = int(args.size, 0) if args.size else None

    syms, preferred = parse_map(args.map)
    if not syms:
        sys.exit(f"{args.map}: no symbols parsed -- is this an MSVC linker "
                 f"map? Check that GenerateMapFile is on in vwifi.vcxproj.")

    rips = list(args.rips)
    if args.stdin:
        blob = sys.stdin.read()
        rips += re.findall(r"(?:RIP=|0x)?([0-9a-fA-F]{8,16})\b", blob)

    if not rips:
        sys.exit("no addresses given")

    print(f"{len(syms)} symbols from {args.map}"
          + (f", preferred base 0x{preferred:x}" if preferred else ""))
    print(f"image base 0x{base:x}"
          + (f", size 0x{size:x}" if size else "") + "\n")

    for r in rips:
        try:
            rip = int(r, 16) if not str(r).startswith("0x") else int(r, 0)
        except ValueError:
            continue

        if rip < base or (size is not None and rip >= base + size):
            print(f"0x{rip:016x}  -- outside vwifi.sys; another module")
            continue

        rva = rip - base
        hit = resolve(syms, rva)
        if hit is None:
            print(f"0x{rip:016x}  RVA 0x{rva:x}  -- before the first symbol")
            continue

        addr, name, obj = hit
        print(f"0x{rip:016x}  RVA 0x{rva:<8x}  {demangle(name)}"
              f"+0x{rva - addr:x}   [{obj}]")


if __name__ == "__main__":
    main()
