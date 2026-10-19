#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
DEFAULT_WORLD_DIR="$SCRIPT_DIR/worlds/survey_worlds"
WORLD_DIR="${1:-$DEFAULT_WORLD_DIR}"
SETUP_FILE="$WORKSPACE_ROOT/install/setup.bash"
launch_pid=""

if (( $# > 1 )); then
  printf 'Usage: %s [WORLD_DIR]\n' "${0##*/}" >&2
  exit 2
fi

if [[ ! -f "$SETUP_FILE" ]]; then
  printf 'Workspace setup file not found: %s\n' "$SETUP_FILE" >&2
  exit 1
fi
source "$SETUP_FILE"

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ros2 was not found after sourcing %s\n' "$SETUP_FILE" >&2
  exit 1
fi
if ! command -v setsid >/dev/null 2>&1; then
  printf 'setsid is required to stop each launch without exiting this script.\n' >&2
  exit 1
fi
if [[ ! -d "$WORLD_DIR" ]]; then
  printf 'World directory not found: %s\n' "$WORLD_DIR" >&2
  exit 1
fi

stop_current_launch() {
  if [[ -n "$launch_pid" ]]; then
    trap '' INT
    kill -INT -- "-$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
    sleep 0.5
    kill -TERM -- "-$launch_pid" 2>/dev/null || kill -TERM "$launch_pid" 2>/dev/null || true
    sleep 0.5
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -KILL -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
    launch_pid=""
  fi
}
handle_interrupt() {
  stop_current_launch
  printf '\nInterrupted; stopping survey-world playback.\n' >&2
  exit 130
}
handle_termination() {
  trap - TERM
  stop_current_launch
  exit 143
}
trap handle_interrupt INT
trap handle_termination TERM
trap stop_current_launch EXIT

read -r -p 'Index of the first world: ' first_input
read -r -p 'Index of the last world: ' last_input

if [[ ! "$first_input" =~ ^[0-9]+$ || ! "$last_input" =~ ^[0-9]+$ ]]; then
  printf 'Indexes must be non-negative integers.\n' >&2
  exit 1
fi

first_index=$((10#$first_input))
last_index=$((10#$last_input))
if (( first_index > last_index )); then
  printf 'The first index must not be greater than the last index.\n' >&2
  exit 1
fi

shopt -s nullglob
all_worlds=("$WORLD_DIR"/*.world)
if (( ${#all_worlds[@]} > 0 )); then
  mapfile -t all_worlds < <(printf '%s\n' "${all_worlds[@]}" | LC_ALL=C sort)
fi

world_count=${#all_worlds[@]}
if (( world_count == 0 )); then
  printf 'No world files found in %s.\n' "$WORLD_DIR" >&2
  exit 1
fi
if (( last_index >= world_count )); then
  printf 'Index %d is out of range; found %d world files (valid indexes: 0 to %d).\n' \
    "$last_index" "$world_count" "$((world_count - 1))" >&2
  exit 1
fi

worlds=()
for ((world_index = first_index; world_index <= last_index; world_index++)); do
  worlds+=("${all_worlds[$world_index]}")
done

printf 'Selected %d worlds at array indexes %d to %d.\n' "${#worlds[@]}" "$first_index" "$last_index"

for position in "${!worlds[@]}"; do
  world_path=${worlds[$position]}
  printf '\nLaunching %s\n' "${world_path##*/}"
  setsid ros2 launch socnav_gazebo_reproducer socnav_gazebo_world.launch.py "world:=$world_path" &
  launch_pid=$!
  if wait "$launch_pid"; then
    launch_status=0
  else
    launch_status=$?
  fi
  launch_pid=""

  if (( launch_status != 0 && launch_status != 130 )); then
    printf 'Launch exited with status %d; continuing to the next world.\n' "$launch_status" >&2
  fi
done

printf '\nFinished selected survey worlds.\n'