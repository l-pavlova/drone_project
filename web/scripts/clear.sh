#!/usr/bin/env bash
#
# PARKDRONE clear — wipe a survey area so it can be flown and scored again.
#
#   pnpm clear                       # list what is stored, delete nothing
#   pnpm clear fmi_block             # DB rows + object-store images + captures
#   pnpm clear fmi_block --yes       # no confirmation prompt (for scripts)
#   pnpm clear fmi_block --db-only   # keep sim/output/<area>/ on disk
#   pnpm clear fmi_block --disk-only # keep the database, drop the captures
#   pnpm clear fmi_block --force     # allow it even for a scored golden fixture
#   c           # EVERY stored area, then sweep the leftovers
#
# --all is what a demo wants, and a per-area clear is not the same thing.
# `bay_state` has no survey_area column — a bay has one current answer, whoever
# saw it — and this project's areas deliberately overlap (fmi_block,
# fmi_block_4st and dji_0035 all cover the FMI block). So clearing one area
# leaves the others' colours painted on the same block, and a freshly cleared
# world opens onto the previous flight's map, which looks exactly like the new
# flight having already finished.
#
# Why you need this before re-flying: ingest is idempotent on
# (drone_id, survey_area, frame_idx), so a second flight of an area already in
# the database posts duplicates — no classify job, no bay_state change, no
# WebSocket delta, and a map that never repaints while the drone flies.
#
# The work happens in parkdrone_vision/clear_area.py (that is where the ordering
# and the guardrails are documented); this wrapper exists so it is one command
# from web/ and so it finds the same Python the rest of the stack uses.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PYTHON:-python}"
command -v "$PY" >/dev/null 2>&1 || {
  echo "error: python not found (set PYTHON=/path/to/python)" >&2; exit 1; }

cd "$ROOT/apps/vision-worker"
exec "$PY" -m parkdrone_vision.clear_area "$@"
