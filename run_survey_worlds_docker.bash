#!/usr/bin/env bash
# Runs each survey world in a fresh Docker container, removing it afterwards,
# so a failed Gazebo run cannot affect the following ones.
# Usage: run_survey_worlds_docker.bash [WORLD_DIR_INSIDE_CONTAINER]

set -o pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# Workspace root on the host: <ws>/src/socnav_gazebo_reproducer/<script>
HOST_WS=$(cd -- "$SCRIPT_DIR/../.." && pwd)
IMAGE="pmb2_hunavsim"
DEFAULT_WORLD_DIR="/home/hunav_gz_classic_ws/src/socnav_gazebo_reproducer/worlds/survey_worlds"
WORLD_DIR="${1:-$DEFAULT_WORLD_DIR}"
TIMEOUT_SECONDS=200
MIN_RUNTIME_SECONDS=6 #10
LINGERING_EXIT_CODE=99
CONTAINER_NAME="hunavsim_pmb2_survey_$$"

if (( $# > 1 )); then
  printf 'Usage: %s [WORLD_DIR_INSIDE_CONTAINER]\n' "${0##*/}" >&2
  exit 2
fi

SOURCES='source /opt/ros/humble/setup.bash
source /home/pmb2_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
source /home/hunav_gz_classic_ws/install/setup.bash'

docker_run() {
  local tty_flag=()
  [[ -t 0 ]] && tty_flag=(-t)
  docker run -i "${tty_flag[@]}" --rm --init \
    --name "$CONTAINER_NAME" \
    --gpus all \
    --runtime=nvidia \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --privileged \
    --mount type=bind,source="$HOST_WS",target=/home/hunav_gz_classic_ws \
    --entrypoint bash \
    "$IMAGE" \
    -c "$SOURCES
$1" bash "${@:2}"
}

cleanup() {
  docker stop -t 10 "$CONTAINER_NAME" >/dev/null 2>&1 || true
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
failed_worlds=()
# Failures are also appended here as they happen, so they survive a crashed run.
FAILED_LOG="$SCRIPT_DIR/survey_failed_worlds_$(date +%Y%m%d_%H%M%S).log"
executed=0
world_count=0

print_summary() {
  printf '\n========== Summary ==========\n'
  printf 'Worlds found: %d, executed: %d, failed: %d\n' "$world_count" "$executed" "${#failed_worlds[@]}"
  if (( ${#failed_worlds[@]} > 0 )); then
    printf 'Failed worlds:\n'
    printf '  - %s\n' "${failed_worlds[@]}"
    printf 'Failures also saved in: %s\n' "$FAILED_LOG"
  fi
}

trap 'cleanup; printf "\nInterrupted.\n" >&2; print_summary; exit 130' INT
trap 'cleanup; exit 143' TERM

xhost +local:docker >/dev/null 2>&1 || true

CONTAINER_WS="/home/hunav_gz_classic_ws"
if [[ "$WORLD_DIR" != "$CONTAINER_WS"/* ]]; then
  printf 'WORLD_DIR must be inside %s (container path).\n' "$CONTAINER_WS" >&2
  exit 1
fi
HOST_WORLD_DIR="$HOST_WS${WORLD_DIR#"$CONTAINER_WS"}"
if [[ ! -d "$HOST_WORLD_DIR" ]]; then
  printf 'World directory not found: %s\n' "$HOST_WORLD_DIR" >&2
  exit 1
fi

# List worlds from the host through the bind mount and map them to container paths
all_worlds=()
while IFS= read -r f; do
  all_worlds+=("$WORLD_DIR/${f##*/}")
done < <(printf '%s\n' "$HOST_WORLD_DIR"/*.world | LC_ALL=C sort)
world_count=${#all_worlds[@]}
if (( world_count == 0 )) || [[ "${all_worlds[0]}" == *"*.world" ]]; then
  printf 'No world files found in %s (inside the container).\n' "$WORLD_DIR" >&2
  exit 1
fi
printf 'Found %d worlds in %s\n' "$world_count" "$WORLD_DIR"

read -r -p 'Index of the first world: ' first_input
read -r -p 'Index of the last world: ' last_input
if [[ ! "$first_input" =~ ^[0-9]+$ || ! "$last_input" =~ ^[0-9]+$ ]]; then
  printf 'Indexes must be non-negative integers.\n' >&2
  exit 1
fi
first_index=$((10#$first_input))
last_index=$((10#$last_input))
if (( first_index > last_index || last_index >= world_count )); then
  printf 'Invalid range (valid indexes: 0 to %d, first <= last).\n' "$((world_count - 1))" >&2
  exit 1
fi

selected_count=$((last_index - first_index + 1))
for ((i = first_index; i <= last_index; i++)); do
  world_path=${all_worlds[$i]}
  printf '\n===== Running scenario %d/%d =====\n' "$((i - first_index + 1))" "$selected_count"
  printf '[%d] Launching %s in a fresh container\n' "$i" "${world_path##*/}"
  inner='timeout -s INT -k 15 '"$TIMEOUT_SECONDS"' ros2 launch socnav_gazebo_reproducer socnav_gazebo_world.launch.py "world:=$1"
rc=$?
if pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null; then
  echo "WARNING: Gazebo processes still alive after launch ended; killing them." >&2
  pkill -9 -x gzserver; pkill -9 -x gzclient
  (( rc == 0 )) && rc='"$LINGERING_EXIT_CODE"'
fi
exit $rc'
  start_time=$SECONDS
  log_file=$(mktemp)
  docker_run "$inner" "$world_path" 2>&1 | tee "$log_file"
  status=${PIPESTATUS[0]}
  elapsed=$((SECONDS - start_time))
  cleanup
  executed=$((executed + 1))

  # The plugin logs "Trajectory step X / Y" every 20 steps; the last one logged
  # when the trajectory is fully played satisfies X + 20 > Y.
  last_progress=$(tr -d '\r' < "$log_file" | grep -aoE 'Trajectory step [0-9]+ / [0-9]+' | tail -n 1)
  rm -f "$log_file"
  reasons=()
  if [[ -z "$last_progress" ]]; then
    reasons+=("no trajectory progress reported")
  else
    read -r _ _ step_x _ step_y <<< "$last_progress"
    if (( step_x + 20 <= step_y )); then
      reasons+=("trajectory incomplete: step $step_x / $step_y")
    fi
  fi
  if (( status == 124 )); then
    reasons+=("timeout")
  elif (( status == LINGERING_EXIT_CODE )); then
    reasons+=("Gazebo processes left alive")
  elif (( status != 0 )); then
    reasons+=("exit status $status")
  fi
  if (( elapsed < MIN_RUNTIME_SECONDS )); then
    reasons+=("ended too early")
  fi

  if (( ${#reasons[@]} > 0 )); then
    reason_text=$(IFS=,; printf '%s' "${reasons[*]}")
    printf '\nFAILED: %s (%s, %ds)\n' "${world_path##*/}" "${reason_text//,/, }" "$elapsed" >&2
    failed_worlds+=("[$i] ${world_path##*/} (${reason_text//,/, }; ${elapsed}s)")
    printf '%s\n' "${failed_worlds[-1]}" >> "$FAILED_LOG"
  fi
done

printf '\nFinished selected survey worlds.\n'
print_summary
(( ${#failed_worlds[@]} == 0 ))
