#!/usr/bin/env bash
# setup_potree.sh — One-time download of PotreeConverter 2.x (includes its own viewer).
#
# Creates:
#   vendor/PotreeConverter/   (binary + resources/page_template/libs/)
#
# Run from the repo root:
#   bash scripts/setup_potree.sh
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

VENDOR_DIR="$REPO_ROOT/vendor/PotreeConverter"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

if [ -x "$VENDOR_DIR/PotreeConverter" ] && [ -d "$VENDOR_DIR/resources/page_template/libs" ]; then
    echo "PotreeConverter already installed at $VENDOR_DIR"
else
    echo "Downloading PotreeConverter 2.1.1 …"
    mkdir -p "$VENDOR_DIR"
    curl -fL \
        "https://github.com/potree/PotreeConverter/releases/download/2.1.1/PotreeConverter_2.1.1_x64_linux.zip" \
        -o "$TMP/PotreeConverter.zip"

    # Extract everything (binary + resources/page_template with bundled viewer libs)
    python3 - <<'PYEOF'
import zipfile, os, shutil, sys
src = os.environ.get("ZIP") or sys.argv[1]
dest = os.environ.get("DEST") or sys.argv[2]
strip = "PotreeConverter_linux_x64/"
with zipfile.ZipFile(src) as z:
    for member in z.infolist():
        rel = member.filename
        if rel.startswith(strip):
            rel = rel[len(strip):]
        if not rel:
            continue
        out = os.path.join(dest, rel)
        if member.is_dir():
            os.makedirs(out, exist_ok=True)
        else:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with z.open(member) as sf, open(out, "wb") as df:
                shutil.copyfileobj(sf, df)
PYEOF
    ZIP="$TMP/PotreeConverter.zip" DEST="$VENDOR_DIR" python3 - <<'PYEOF'
import zipfile, os, shutil
src = os.environ["ZIP"]
dest = os.environ["DEST"]
strip = "PotreeConverter_linux_x64/"
with zipfile.ZipFile(src) as z:
    for member in z.infolist():
        rel = member.filename
        if rel.startswith(strip):
            rel = rel[len(strip):]
        if not rel:
            continue
        out = os.path.join(dest, rel)
        if member.is_dir():
            os.makedirs(out, exist_ok=True)
        else:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with z.open(member) as sf, open(out, "wb") as df:
                shutil.copyfileobj(sf, df)
PYEOF

    chmod +x "$VENDOR_DIR/PotreeConverter"

    # The bundled liblaszip.so won't resolve on this system; symlink the installed one
    LASZIP_SO=$(ldconfig -p | grep 'liblaszip\.so\.' | awk '{print $NF}' | head -1)
    if [ -n "$LASZIP_SO" ]; then
        ln -sf "$LASZIP_SO" "$VENDOR_DIR/liblaszip.so"
    fi

    echo "  → $VENDOR_DIR/PotreeConverter"
    echo "  → $VENDOR_DIR/resources/page_template/libs  ($(ls $VENDOR_DIR/resources/page_template/libs | wc -l) libs)"
fi

echo ""
echo "Done. Usage:"
echo "  bash scripts/serve_potree.sh <session-name>"
