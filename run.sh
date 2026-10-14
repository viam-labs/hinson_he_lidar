#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ ! -d deps/viam ]]; then
  mkdir -p deps
  export PYTHONPATH="$(pwd)/deps${PYTHONPATH:+:$PYTHONPATH}"
  if ! python3 -m pip --version >/dev/null 2>&1; then
    get_pip="$(mktemp)"
    python3 -c 'import urllib.request, sys; urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py", sys.argv[1])' "$get_pip"
    python3 "$get_pip" --target "$(pwd)/deps"
    rm -f "$get_pip"
  fi
  python3 -m pip install --upgrade --target deps -r requirements.txt
fi

export PYTHONPATH="$(pwd)/deps:$(pwd)/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m hinson_he.main "$@"
