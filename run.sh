#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -d deps ]]; then
  python3 -m pip install --target deps -r requirements.txt
fi
export PYTHONPATH="$(pwd)/deps:$(pwd)/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m hinson_he.main "$@"
