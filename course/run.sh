#!/usr/bin/env bash
# Milestone 6.2 course harness. Host-native: no container, no Chrono, no
# Unreal. The course is DERIVED from the vehicle, so it is regenerated rather
# than edited -- gen_innova.py's rule, applied one level up.
set -euo pipefail
cd "$(dirname "$0")"

case "${1:-help}" in
  gen)   exec python3 gen_course.py ;;
  plot)  python3 gen_course.py >/dev/null; exec python3 plot_course.py "${2:-118}" ;;
  check) # regenerate and confirm the JSON is byte-identical: catches a
         # hand-edit of the generated file, the one thing the contract forbids.
         cp simA_course.json /tmp/course_before.json 2>/dev/null || true
         python3 gen_course.py >/dev/null
         if diff -q /tmp/course_before.json simA_course.json >/dev/null 2>&1; then
             echo "simA_course.json is up to date with gen_course.py"
         else
             echo "simA_course.json REGENERATED -- it was stale or hand-edited"
             exit 1
         fi ;;
  *) echo "usage: run.sh gen|plot [cols]|check" ;;
esac
