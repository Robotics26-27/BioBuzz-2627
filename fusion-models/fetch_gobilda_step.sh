#!/bin/sh
# Download real goBILDA STEP zips for every BOM SKU into ./gobilda_step/ (git-ignored).
# Note: the Strafer kit zip is large (~157 MB STEP).
set -e
cd "$(dirname "$0")"
mkdir -p gobilda_step
for sku in 1117-0040-0352 1117-0088-0352 1120-0005-0144 1120-0010-0264 1120-0013-0336 1201-0043-0002 1310-0016-4008 1611-0514-4008 2106-4008-2880 2106-4008-3840 3209-0001-0007 3422-4008-0020 3423-0006-0184 3423-0006-0264 3423-0006-0328 3423-0006-0514 3614-0014-0096 5203-2402-0001 5203-2402-0014; do
  curl -fL --retry 2 -o "gobilda_step/$sku.zip" "https://www.gobilda.com/content/step_files/$sku.zip" || echo "failed: $sku"
done
