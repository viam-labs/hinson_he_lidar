#!/usr/bin/env bash
# Package a per-architecture module archive. deps/ is installed on the build
# machine so the tarball matches linux/amd64 or linux/arm64 Viam builders.
set -euo pipefail
cd "$(dirname "$0")"
rm -rf deps dist
python3 -m pip install --target deps -r requirements.txt
mkdir -p dist
tar czf dist/archive.tar.gz run.sh requirements.txt setup.sh src deps meta.json README.md
