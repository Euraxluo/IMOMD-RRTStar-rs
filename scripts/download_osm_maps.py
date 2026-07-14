#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["gdown>=5"]
# ///
"""Download large OSM maps for IMOMD-RRTStar validation.

Seattle.osm and sanfrancisco_bugtrap.osm are hosted on Google Drive (not in Git).
Smaller maps (FRB, quincy, sanf, etc.) are already in the C++ reference repo.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OSM_DIR = ROOT / "tmp" / "imomd-cpp" / "osm_data"
DRIVE_FOLDER = "https://drive.google.com/drive/folders/1sA5MH-K6EoiYh0PNqJMcITsmXCxjyfdJ?usp=sharing"

GITHUB_MAPS = {
    "quincy.osm": "https://raw.githubusercontent.com/UMich-BipedLab/IMOMD-RRTStar/lib_isrr_release/osm_data/quincy.osm",
    "sanf.osm": "https://raw.githubusercontent.com/UMich-BipedLab/IMOMD-RRTStar/lib_isrr_release/osm_data/sanf.osm",
    "san_pablo.osm": "https://raw.githubusercontent.com/UMich-BipedLab/IMOMD-RRTStar/lib_isrr_release/osm_data/san_pablo.osm",
}


def download_github_maps(names: list[str]) -> int:
    import urllib.request

    OSM_DIR.mkdir(parents=True, exist_ok=True)
    ok = 0
    for name in names:
        url = GITHUB_MAPS[name]
        dest = OSM_DIR / name
        if dest.exists():
            print(f"skip {name}: already exists")
            ok += 1
            continue
        print(f"downloading {name} ...")
        urllib.request.urlretrieve(url, dest)
        print(f"saved {dest} ({dest.stat().st_size} bytes)")
        ok += 1
    return ok


def download_drive_folder() -> bool:
    try:
        import gdown
    except ImportError:
        print("gdown not installed; run: uv pip install gdown", file=sys.stderr)
        return False

    OSM_DIR.mkdir(parents=True, exist_ok=True)
    print(f"attempting Google Drive folder download into {OSM_DIR}")
    try:
        gdown.download_folder(DRIVE_FOLDER, output=str(OSM_DIR), quiet=False, use_cookies=False)
        return True
    except Exception as exc:
        print(f"Google Drive download failed: {exc}", file=sys.stderr)
        print(
            "Manually download Seattle.osm and sanfrancisco_bugtrap.osm from:\n"
            f"  {DRIVE_FOLDER}\n"
            f"and place them in {OSM_DIR}",
            file=sys.stderr,
        )
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--github",
        nargs="*",
        choices=sorted(GITHUB_MAPS),
        default=["quincy.osm"],
        help="download maps available on GitHub",
    )
    parser.add_argument(
        "--drive",
        action="store_true",
        help="attempt Google Drive bundle (Seattle.osm, sanfrancisco_bugtrap.osm)",
    )
    args = parser.parse_args()

    download_github_maps(args.github)
    if args.drive:
        download_drive_folder()

    seattle = OSM_DIR / "Seattle.osm"
    if seattle.exists():
        print(f"Seattle.osm ready: {seattle}")
    else:
        print("Seattle.osm missing — use --drive or manual download for full paper reproduction")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
