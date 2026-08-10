#!/usr/bin/env bash
#
# Stop the warehouse simulation and clean up every sim-related process.
# Safe to run repeatedly; exits 0 even if nothing was running.
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$SCRIPT_DIR/.sim"

# Patterns matched against the full command line of *this user's* processes.
PATTERNS=(
  "gz sim"
  "gz-sim-server"
  "gz-sim-gui"
  "ruby .*gz sim"
  "ros_gz_bridge"
  "parameter_bridge"
  "static_transform_publisher"
)

# Never signal ourselves or any of our ancestors: a caller's command line may
# happen to contain one of the patterns above (e.g. a grep for leftovers).
build_exclusions() {
  local pid=$$
  EXCLUDE=()
  while [[ -n "$pid" && "$pid" != "0" && "$pid" != "1" ]]; do
    EXCLUDE+=("$pid")
    pid="$(awk '{print $4}' "/proc/$pid/stat" 2>/dev/null)" || break
  done
}
build_exclusions

# Filter a newline-separated pid list, dropping excluded pids.
filter_pids() {
  local pids="$1" out=() p
  for p in $pids; do
    local skip=0 e
    for e in "${EXCLUDE[@]}"; do [[ "$p" == "$e" ]] && skip=1 && break; done
    [[ $skip -eq 0 ]] && out+=("$p")
  done
  printf '%s\n' "${out[@]:-}" | grep -v '^$' || true
}

matching_pids() {
  filter_pids "$(pgrep -u "$(id -u)" -f "$1" 2>/dev/null || true)"
}

kill_pidfiles() {
  local sig="$1" found=0
  shopt -s nullglob
  for f in "$RUN_DIR"/*.pid; do
    local pid
    pid="$(cat "$f" 2>/dev/null)" || continue
    [[ -n "$pid" ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "  $sig -> PID $pid ($(basename "$f" .pid))"
      # Kill the whole process group where possible (gz spawns children).
      kill "-$sig" "-$pid" 2>/dev/null || kill "-$sig" "$pid" 2>/dev/null
      found=1
    fi
  done
  shopt -u nullglob
  return $((1 - found))
}

kill_patterns() {
  local sig="$1" found=0
  for pat in "${PATTERNS[@]}"; do
    local pids
    pids="$(matching_pids "$pat")"
    if [[ -n "$pids" ]]; then
      echo "  $sig -> [$pat] $(echo "$pids" | tr '\n' ' ')"
      # shellcheck disable=SC2086
      kill "-$sig" $pids 2>/dev/null
      found=1
    fi
  done
  return $((1 - found))
}

remaining() {
  local out=""
  for pat in "${PATTERNS[@]}"; do
    local pids
    pids="$(matching_pids "$pat")"
    [[ -n "$pids" ]] && out+="$pids"$'\n'
  done
  echo "$out" | grep -v '^$' | sort -u
}

echo "==> Stopping simulation (graceful)"
kill_pidfiles TERM
kill_patterns TERM

for _ in $(seq 1 10); do
  [[ -z "$(remaining)" ]] && break
  sleep 0.5
done

if [[ -n "$(remaining)" ]]; then
  echo "==> Some processes ignored SIGTERM, forcing"
  kill_pidfiles KILL
  kill_patterns KILL
  sleep 1
fi

rm -f "$RUN_DIR"/*.pid 2>/dev/null

LEFT="$(remaining)"
if [[ -n "$LEFT" ]]; then
  echo "WARNING: still alive after SIGKILL:" >&2
  ps -o pid,cmd -p $(echo "$LEFT" | tr '\n' ',' | sed 's/,$//') >&2
  exit 1
fi

echo "==> Clean. No Gazebo or bridge processes remaining."
