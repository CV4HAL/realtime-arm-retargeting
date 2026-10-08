#!/usr/bin/env bash
# CV4HAL single entrypoint. Run ./run.sh --help for usage.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="uf_software"
IMAGE="danielwang123321/uf-ubuntu-docker"

usage() {
  cat <<'EOF'
Usage: ./run.sh <command> [flags]

Commands:
  check       Verify the Python environment and camera
  pose        Basic body pose tracking
  cv          Holistic tracking (pose + fingers + FPS)
  sim         CV bridge to the UFACTORY Lite 6 (simulator or real arm)
  sim-up      Start the UFACTORY simulator in Docker
  sim-down    Stop the UFACTORY simulator
  test        Run pytest and ruff

Run ./run.sh <command> --help for the flags of a command.

Examples:
  ./run.sh check
  ./run.sh sim-up
  ./run.sh sim --dry-run
  ./run.sh sim --dual-hand --swap-hands
  ./run.sh cv --camera 1
EOF
}

usage_cv() {
  cat <<'EOF'
Usage: ./run.sh cv [--camera N]

  --camera N   camera index (default 0)
EOF
}

usage_sim() {
  cat <<'EOF'
Usage: ./run.sh sim [flags]

  --ip IP        robot or simulator IP (default 127.0.0.1)
  --camera N     camera index (default 0)
  --rate HZ      servo rate in Hz
  --dual-hand    left hand sets the TCP orientation, closed fist freezes it
  --swap-hands   swap the left and right hand assignment
  --dry-run      track and compute targets without connecting to the robot
EOF
}

die() {
  echo "run.sh: $*" >&2
  exit 2
}

fail() {
  echo "run.sh: $*" >&2
  exit 1
}

find_python() {
  if [[ -x "$ROOT/.venv/bin/python" ]]; then
    PYTHON="$ROOT/.venv/bin/python"
  elif command -v python3.11 >/dev/null 2>&1; then
    PYTHON="$(command -v python3.11)"
  else
    fail "Python 3.11 not found. Create the venv (python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt) or install python3.11."
  fi
}

need_docker() {
  command -v docker >/dev/null 2>&1 || fail "docker not found. Install Docker Engine (see SIMULATOR.md)."
  docker info >/dev/null 2>&1 || fail "cannot talk to the Docker daemon. Is it running, and is your user in the 'docker' group? (see SIMULATOR.md)"
}

# Fail early when a flag that takes a value has none.
require_value() {
  [[ $# -ge 2 && "$2" != --* ]] || die "$1 requires a value. See ./run.sh ${CMD} --help"
}

wants_help() {
  local a
  for a in "$@"; do
    [[ "$a" == "--help" || "$a" == "-h" ]] && return 0
  done
  return 1
}

# Validate flags against a whitelist; flags that take a value are listed in $VALUE_FLAGS.
check_flags() {
  local value_flags="$1" bool_flags="$2"
  shift 2
  while [[ $# -gt 0 ]]; do
    if [[ " $value_flags " == *" $1 "* ]]; then
      require_value "$@"
      shift 2
    elif [[ " $bool_flags " == *" $1 "* ]]; then
      shift
    else
      die "unknown flag '$1' for '${CMD}'. See ./run.sh ${CMD} --help"
    fi
  done
}

cmd_sim_up() {
  need_docker
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker start "$CONTAINER" >/dev/null
  else
    docker pull "$IMAGE"
    docker run -d --name "$CONTAINER" \
      -p 18333:18333 \
      -p 502:502 -p 503:503 -p 504:504 \
      -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
      "$IMAGE" tail -f /dev/null >/dev/null
  fi
  docker exec -d "$CONTAINER" /xarm_scripts/xarm_start.sh 6 9
  echo "Simulator starting. UFACTORY Studio: http://127.0.0.1:18333"
}

cmd_sim_down() {
  need_docker
  docker stop "$CONTAINER" >/dev/null 2>&1 || fail "container '$CONTAINER' is not running."
  echo "Simulator stopped."
}

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

CMD="$1"
shift

case "$CMD" in
  -h|--help|help)
    usage
    ;;
  check|pose|sim-up|sim-down|test)
    if wants_help "$@"; then usage; exit 0; fi
    [[ $# -eq 0 ]] || die "'$CMD' takes no flags. See ./run.sh --help"
    case "$CMD" in
      check) find_python; exec "$PYTHON" "$ROOT/src/check_setup.py" ;;
      pose)  find_python; exec "$PYTHON" "$ROOT/src/pose_tracking.py" ;;
      sim-up) cmd_sim_up ;;
      sim-down) cmd_sim_down ;;
      test)
        find_python
        cd "$ROOT"
        "$PYTHON" -m pytest
        "$PYTHON" -m ruff check .
        ;;
    esac
    ;;
  cv)
    if wants_help "$@"; then usage_cv; exit 0; fi
    check_flags "--camera" "" "$@"
    find_python
    exec "$PYTHON" "$ROOT/src/holistic_tracking.py" "$@"
    ;;
  sim)
    if wants_help "$@"; then usage_sim; exit 0; fi
    check_flags "--ip --camera --rate" "--dual-hand --swap-hands --dry-run" "$@"
    find_python
    exec "$PYTHON" "$ROOT/src/sim_bridge.py" "$@"
    ;;
  *)
    echo "run.sh: unknown command '$CMD'" >&2
    usage >&2
    exit 2
    ;;
esac
