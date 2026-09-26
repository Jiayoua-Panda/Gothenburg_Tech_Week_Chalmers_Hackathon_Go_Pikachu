#!/bin/sh
# Download the third-party G1-DWAQ stair policy weights used by dwaq_policy.py (not stored in this repository).
# Source: https://github.com/liuyufei-nubot/G1DWAQ_Lab (commit bebb0ea, BSD-3-Clause). Not trained by this project.
set -e
cd "$(dirname "$0")"
mkdir -p third_party/g1_dwaq
URL=https://raw.githubusercontent.com/liuyufei-nubot/G1DWAQ_Lab/bebb0ea413f3bc00860481c2f11acf30d9c416cf/TienKung-Lab/logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt
curl -sL -o third_party/g1_dwaq/model_9999.pt "$URL"
EXPECTED=5042017a558b98ab24a3784d1f960383ee42015d79b67b24b74f6b6a117759c1
GOT=$( (shasum -a 256 third_party/g1_dwaq/model_9999.pt 2>/dev/null || sha256sum third_party/g1_dwaq/model_9999.pt) | cut -d' ' -f1)
[ "$GOT" = "$EXPECTED" ] && echo "OK: model_9999.pt (sha256 $GOT)" || { echo "SHA-256 mismatch: $GOT"; exit 1; }
