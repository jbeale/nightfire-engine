#!/bin/sh
# Smoke-test one native build with a locally extracted US PS2 disc.
set -eu

if [ "$#" -ne 2 ]; then
    echo "usage: $0 <build-dir> <extracted-disc-dir>" >&2
    exit 2
fi

build=$1
disc=$2
game="$build/nightfire"
if [ -x "$build/nightfire.app/Contents/MacOS/nightfire" ]; then
    game="$build/nightfire.app/Contents/MacOS/nightfire"
fi
if [ ! -x "$game" ] || [ ! -x "$build/nfdump" ]; then
    echo "Build the nightfire and nfdump targets first." >&2
    exit 2
fi
if [ ! -r "$disc/ACTION.ELF" ] || [ ! -r "$disc/FILES.BIN" ] || [ ! -r "$disc/DRIVING/MIS01.VIV" ]; then
    echo "Use the full extracted PS2 disc directory." >&2
    exit 2
fi

out=$(mktemp -d "${TMPDIR:-/tmp}/nightfire-smoke.XXXXXX")
echo "Smoke test artifacts are in $out"

printf 'Validating disc data... '
"$build/nfdump" "$disc" validate > "$out/validate.log" 2>&1
echo 'ok'

capture() {
    name=$1
    expected=$2
    shift 2
    printf 'Running %s... ' "$name"
    "$game" "$disc" "$@" --mute --shot "$out/$name.bmp" > "$out/$name.log" 2>&1
    if ! grep -Fq "$expected" "$out/$name.log" || [ ! -s "$out/$name.bmp" ]; then
        echo "failed. See $out/$name.log" >&2
        exit 1
    fi
    echo 'ok'
}

capture main-menu 'frontend page 0x40000002 ->' --page 0x40000002 --press wait90 --frames 30
capture skyrail 'frame 300:' --mp 07000024.bin --mode arena --bots 3 --seed 42 --frames 300
capture exchange '07000005.bin: 300 frames' --mission 07000005.bin --difficulty 1 --frames 300
capture paris 'drive paris: 300 ticks' --drive paris --frames 300
