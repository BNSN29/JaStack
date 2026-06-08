#!/usr/bin/env bash
# =============================================================================
# dev_sim.sh  —  Simple simulation launcher for drone delivery development
#
# Usage:
#   ./dev_sim.sh          # launch sim + drop you into the aircraft container
#   ./dev_sim.sh stop     # stop and remove all sim containers
#   ./dev_sim.sh shell    # re-attach to a running aircraft container
#
# =============================================================================

set -euo pipefail

# ── Config — edit these as needed ────────────────────────────────────────────
AUTOPILOT="px4"
WORLD="swiss_town"
NUM_QUADS=1
RTF=1.0          # real-time factor (1.0 = real time, 3.0 = 3x faster)
HEADLESS=false   # set true if you have no display / running headless

AIRCRAFT_CONTAINER="aircraft-container-inst0_1"
SIM_CONTAINER="simulation-container-inst0"

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[dev_sim]${NC} $*"; }
warn()  { echo -e "${YELLOW}[dev_sim]${NC} $*"; }
error() { echo -e "${RED}[dev_sim]${NC} $*"; exit 1; }

# ── Helpers ───────────────────────────────────────────────────────────────────
container_running() { docker ps --format '{{.Names}}' | grep  "^${1}$"; }

wait_for_container() {
  local name="$1" timeout=60 elapsed=0
  info "Waiting for container '${name}' to be ready..."
  until container_running "$name"; do
    sleep 2; elapsed=$((elapsed + 2))
    [[ $elapsed -ge $timeout ]] && error "Timeout waiting for '${name}'"
    echo -n "."
  done
  echo ""
  info "Container '${name}' is up."
}

# ── Stop ──────────────────────────────────────────────────────────────────────
cmd_stop() {
  info "Stopping simulation containers..."
  docker ps --format '{{.Names}}' \
    | grep -E '(aircraft|simulation|ground)-container' \
    | xargs -r docker stop
  docker ps -a --format '{{.Names}}' \
    | grep -E '(aircraft|simulation|ground)-container' \
    | xargs -r docker rm
  info "Done."
}

# ── Shell — re-attach to running aircraft container ───────────────────────────
cmd_shell() {
  if ! container_running "$AIRCRAFT_CONTAINER"; then
    error "Aircraft container '${AIRCRAFT_CONTAINER}' is not running. Run ./dev_sim.sh first."
  fi
  info "Opening shell in '${AIRCRAFT_CONTAINER}'..."
  docker exec -it "$AIRCRAFT_CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    source /aas/github_ws/install/setup.bash
    source /aas/aircraft_ws/install/setup.bash
    export PS1='[aircraft:\w]\$ '
    exec bash
  "
}

# ── Launch ────────────────────────────────────────────────────────────────────
cmd_launch() {
  # Sanity check: must be run from tools_and_docs/
  [[ -f "./sim_run.sh" ]] || error "Run this script from the tools_and_docs/ directory."

  # Stop any leftover containers from a previous session
  if container_running "$SIM_CONTAINER" || container_running "$AIRCRAFT_CONTAINER"; then
    warn "Containers already running — stopping them first..."
    cmd_stop
    sleep 2
  fi

  info "Starting simulation..."
  info "  autopilot : ${AUTOPILOT}"
  info "  world     : ${WORLD}"
  info "  quads     : ${NUM_QUADS}"
  info "  RTF       : ${RTF}"
  info "  headless  : ${HEADLESS}"
  echo ""

  # Launch the original sim_run.sh in the background (it handles docker-compose)
  AUTOPILOT="$AUTOPILOT" \
  NUM_QUADS="$NUM_QUADS" \
  NUM_VTOLS=0 \
  WORLD="$WORLD" \
  HEADLESS="$HEADLESS" \
  RTF="$RTF" \
  CAMERA=false \
  LIDAR=false \
    bash ./sim_run.sh &
  SIM_PID=$!

  # Wait for the aircraft container to come up
  wait_for_container "$AIRCRAFT_CONTAINER"

  # Give PX4 SITL a few seconds to initialize
  info "Waiting 8s for PX4 SITL to initialize..."
  sleep 8

  info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  info "Simulation is ready. Dropping into aircraft container."
  info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  info ""
  info "Useful commands inside the container:"
  echo "  ros2 topic list                            # see all PX4 topics"
  echo "  ros2 topic echo /Drone1/fmu/out/vehicle_status"
  echo "  colcon build --packages-select delivery_mission"
  echo "  ros2 run delivery_mission preflight_check --ros-args -r __ns:=/Drone1 -p use_sim_time:=true"
  echo ""
  info "To stop the sim from another terminal:  ./dev_sim.sh stop"
  echo ""

  cmd_shell
}

# ── Entry point ───────────────────────────────────────────────────────────────
case "${1:-launch}" in
  launch | "")  cmd_launch ;;
  stop)         cmd_stop   ;;
  shell)        cmd_shell  ;;
  *)
    echo "Usage: $0 [launch|stop|shell]"
    exit 1
    ;;
esac