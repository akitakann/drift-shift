#!/bin/bash
# Renders the game's effects and encodes them to assets/audio/sfx/*.ogg.
# Usage: tools/sfx/build.sh [--pickup piro|pon|cha]
# Needs python3 with numpy and scipy, and ffmpeg with libvorbis.
set -euo pipefail
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
python3 -I tools/sfx/designs.py "$work" "$@"
mkdir -p assets/audio/sfx
for wav in "$work"/*.wav; do
    ffmpeg -loglevel error -y -i "$wav" -c:a libvorbis -q:a 5 "assets/audio/sfx/$(basename "${wav%.wav}").ogg"
done
ls -l assets/audio/sfx
