#!/usr/bin/env bash
# =============================================================================
# run.sh - run the Drone-Inspector simulation on an Apple Silicon Mac (M1-M4).
#
# Usage (from the repo folder):
#   ./run.sh                 orbit inspection
#   ./run.sh --sar           search and rescue in the PHOENIX mine
#   ./run.sh --patrol        lawnmower patrol over the predefined area
#   ./run.sh --yolo          any sim.sh option is passed through
#   ./run.sh build           build (or rebuild) the arm64 container image
#   ./run.sh shell           open a shell inside the container
#   ./run.sh stop            stop the running container
#
# What it does: checks the Mac, starts Docker, builds the image on first use,
# opens the Gazebo window through XQuartz when it is running, then starts
# sim.sh inside the arm64 Ubuntu container.
# =============================================================================
set -eu

cd "$(dirname "${BASH_SOURCE[0]}")"
COMPOSE=(docker compose -f docker-compose.mac.yml)
IMAGE="drone-inspector-sim-arm64"

say()  { printf '\033[1;36m[run]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[run]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[run]\033[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- checks
[ "$(uname -s)" = "Darwin" ] || die "This script is for macOS. On Ubuntu/WSL use ./sim.sh instead."
[ "$(uname -m)" = "arm64" ] || die "Apple Silicon (arm64) required; found $(uname -m). Intel Macs need a different setup."

if ! command -v docker >/dev/null 2>&1; then
  die "Docker is not installed. Install Docker Desktop (https://www.docker.com/products/docker-desktop/) or OrbStack (https://orbstack.dev), open it once, then run ./run.sh again."
fi
if ! docker info >/dev/null 2>&1; then
  say "Starting Docker..."
  open -a Docker 2>/dev/null || open -a OrbStack 2>/dev/null || die "Open Docker Desktop or OrbStack, wait until it is running, then run ./run.sh again."
  for _ in $(seq 1 60); do docker info >/dev/null 2>&1 && break; sleep 2; done
  docker info >/dev/null 2>&1 || die "Docker did not start in time."
fi

# ---------------------------------------------------------------- Gazebo window (optional)
VIEW=0
if [ -n "${DISPLAY:-}" ] || pgrep -x XQuartz >/dev/null 2>&1 || [ -d /Applications/XQuartz.app ]; then
  if command -v xhost >/dev/null 2>&1 && xhost +127.0.0.1 >/dev/null 2>&1; then
    VIEW=1
    say "XQuartz found: the Gazebo window will open on this Mac."
  else
    warn "XQuartz is installed but not reachable. Running headless. Start XQuartz and enable 'Allow connections from network clients' to see Gazebo."
  fi
else
  warn "XQuartz not found: running headless (no Gazebo window). Install it with: brew install --cask xquartz"
fi
export VIEW

# ---------------------------------------------------------------- commands
case "${1:-}" in
  stop)
    say "Stopping the container..."
    "${COMPOSE[@]}" down
    exit 0 ;;
  build)
    say "Building the arm64 image (the first build takes a long time and needs about 20 GB)..."
    "${COMPOSE[@]}" build
    exit 0 ;;
  shell)
    "${COMPOSE[@]}" run --rm sim bash ;;
  *)
    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
      say "First run: building the arm64 image (this takes a long time, once)..."
      "${COMPOSE[@]}" build
    fi
    ARGS="$*"
    say "Starting the simulation inside the container: ./sim.sh ${ARGS:-}"
    say "Inside the container, use the control tab for shortcuts such as sar_start or patrol_start."
    # sim.sh starts a tmux session inside the container and attaches to it.
    "${COMPOSE[@]}" run --rm sim bash -lc "./sim.sh ${ARGS}" ;;
esac
