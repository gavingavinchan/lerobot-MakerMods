#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$REPO/.venv/bin/python" "$REPO/examples/viscous/record_teleop.py" "$@"
