#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Download a downtown Chicago OSM extract for the demo.

Prefers the lz4 Overpass mirror (more reachable from restricted networks).
Falls back through additional mirrors. Writes:
  tmp/osm_data/chicago_downtown.osm
"""

from __future__ import annotations

import argparse
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tmp" / "osm_data" / "chicago_downtown.osm"

# Loop + Near North / West Loop. Keep residential out for a faster complete pull.
QUERY = """
[out:xml][timeout:180];
(
  way["highway"~"motorway|trunk|primary|secondary|tertiary|unclassified"](41.86,-87.66,41.91,-87.61);
);
(._;>;);
out body;
"""

MIRRORS = [
    "https://lz4.overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass-api.de/api/interpreter",
]


def download(url: str, timeout: float) -> bytes:
    data = urllib.parse.urlencode({"data": QUERY}).encode()
    req = urllib.request.Request(
        url,
        data=data,
        headers={"User-Agent": "IMOMD-RRTStar-rs/0.1 (chicago demo map)"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def validate(body: bytes) -> tuple[bool, str]:
    if b"<osm" not in body[:200]:
        return False, "not OSM XML"
    if b"</osm>" not in body[-500:]:
        return False, "incomplete (missing </osm>)"
    text = body.decode("utf-8", errors="replace")
    nodes = text.count("<node ")
    ways = text.count("<way ")
    if ways < 50:
        return False, f"too few ways ({ways})"
    if nodes < 200:
        return False, f"too few nodes ({nodes})"
    return True, f"nodes={nodes} ways={ways}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--timeout", type=float, default=240.0)
    args = parser.parse_args()

    OUT.parent.mkdir(parents=True, exist_ok=True)
    if OUT.exists() and not args.force:
        ok, detail = validate(OUT.read_bytes())
        if ok:
            print(f"skip: {OUT} already valid ({detail})")
            return 0
        print(f"existing file invalid ({detail}); re-downloading")

    last_error = "no mirrors tried"
    for url in MIRRORS:
        print(f"trying {url} ...")
        t0 = time.time()
        try:
            body = download(url, args.timeout)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            last_error = str(exc)
            print(f"  failed: {exc}")
            continue
        ok, detail = validate(body)
        if not ok:
            print(f"  invalid payload: {detail}")
            last_error = detail
            continue
        OUT.write_bytes(body)
        print(f"saved {OUT} ({len(body)} bytes, {detail}) in {time.time() - t0:.1f}s")
        return 0

    print(f"download failed: {last_error}", file=sys.stderr)
    print("Demo can still use synthetic map key `chicago_mega`.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
