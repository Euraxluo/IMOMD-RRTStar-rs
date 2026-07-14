#!/usr/bin/env bash
# Run V2X demo using the project root venv (IMOMD_RRTStar must be built first).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ ! -x .venv/bin/python ]]; then
  echo "Create venv first: uv venv .venv"
  exit 1
fi

uv pip install --python .venv/bin/python fastapi "uvicorn[standard]" pydantic
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 .venv/bin/maturin develop --features python,extension-module -q

export DEMO_OSM_PATH="${DEMO_OSM_PATH:-$ROOT/tmp/imomd-cpp/osm_data/FRB2.osm}"
echo "Demo map: $DEMO_OSM_PATH"
echo "Open http://127.0.0.1:8000"

exec .venv/bin/uvicorn app.main:app --app-dir demo --host 127.0.0.1 --port 8000 --reload
