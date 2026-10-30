# Game audio credits

The engine and tire recordings below replace those two synthesized layers. Impact, wind and landing cues remain original program synthesis.

## sfx/*.ogg

The item pickup (`item_0.ogg`–`item_7.ogg`), rush of air (`whoosh_item.ogg`, `whoosh_boost.ogg`), afterfire (`boost_1.ogg`–`boost_3.ogg`), countdown, sector, lap and record effects are original synthesis made for this game by `tools/sfx/designs.py`; rebuild them with `tools/sfx/build.sh`. The charge, the boost and an item's push are not files: the game revs `engine-c.wav` below itself, higher and louder, so they carry its attribution ("Car Engine Loop" by qubodup, CC BY 4.0).

These sources were obtained as the public high-quality preview MP3s on their individual Freesound pages; they are not the original uncompressed uploads.

## engine-c.wav

"Car Engine Loop" by qubodup
Source: https://freesound.org/people/qubodup/sounds/147242/
License: CC BY 4.0 — https://creativecommons.org/licenses/by/4.0/
Changes: Excerpt 0.00–4.00 seconds; decode/resample to 44.1kHz stereo PCM16 WAV; 50ms loop crossfade; fixed gain +10.86dB.

## tire-a.wav

"Chrysler LHS tire squeal 04 (04-25-2009).wav" by audible-edge
Source: https://freesound.org/people/audible-edge/sounds/71739/
License: CC0 1.0 — http://creativecommons.org/publicdomain/zero/1.0/
Changes: Excerpt 8.00–14.00 seconds; decode/resample to 44.1kHz stereo PCM16 WAV; 50ms loop crossfade; fixed gain +2.31dB; 180Hz high-pass to reduce engine/bass contamination.
