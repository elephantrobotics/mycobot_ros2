#!/usr/bin/env bash
# Install common Python runtime deps for mycobot_ros2 scripts
# (pymycobot API + packaging for version checks).
#
# Usage (after clone):
#   cd ~/colcon_ws/src/mycobot_ros2
#   bash install_deps.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ_FILE="${SCRIPT_DIR}/requirements.txt"

echo "[mycobot_ros2] Installing Python dependencies..."

if ! command -v pip3 >/dev/null 2>&1; then
  echo "ERROR: pip3 not found. Install with: sudo apt-get install -y python3-pip" >&2
  exit 1
fi

if [[ -f "${REQ_FILE}" ]]; then
  pip3 install --user -r "${REQ_FILE}"
else
  pip3 install --user "pymycobot>=4.0.5" packaging
fi

# Optional apt fallback for packaging
if ! python3 -c "import packaging" >/dev/null 2>&1; then
  echo "[mycobot_ros2] packaging still missing; trying apt python3-packaging..."
  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -qq
    sudo apt-get install -y python3-packaging
  fi
fi

python3 - <<'PY'
import sys
print("Python:", sys.version.split()[0])
try:
    import packaging
    print("packaging: OK", getattr(packaging, "__version__", ""))
except Exception as e:
    print("packaging: FAIL", e)
    sys.exit(1)
try:
    import pymycobot
    print("pymycobot: OK", pymycobot.__version__)
except Exception as e:
    print("pymycobot: FAIL", e)
    sys.exit(1)
print("[mycobot_ros2] Dependencies ready.")
PY
