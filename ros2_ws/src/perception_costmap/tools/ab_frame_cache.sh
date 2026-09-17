#!/bin/bash
# A/B the per-frame segmentation cache against the SAME running lean cameras.
#
# Bring the cameras up ONCE first -- they must not restart between variants:
#   ros2 launch perception_costmap perception_stack.launch.py \
#        cameras:=front,left,right costmap:=false
#
#   tools/ab_frame_cache.sh [seconds]      default 60 s, order nocache/cache/nocache
#
# Only the costmap node is started and stopped here, and only the one this
# script started (killed by its own PID). Cameras and sensors are untouched.
# Paths below are dinosaur's. Override AB_DIR / CFG / IGVC for another machine.
#
#   AB_DIR/<variant>/ros2_ws/install   one built workspace per variant, e.g. two
#                                      git worktrees that differ in one commit
#   ORDER                              which variants to run, in order; repeat a
#                                      name to interleave (the drift control)
#
# no `set -u`: ROS's setup.bash reads unset AMENT_* variables and would abort
T=${1:-60}
ORDER=${ORDER:-"nocache cache nocache"}
AB_DIR=${AB_DIR:-/home/dinosaur/chris_test/ab}
IGVC=${IGVC:-/home/dinosaur/IGVC}
CFG=${CFG:-$AB_DIR/nocache/ros2_ws/src/perception_costmap/config/perception_dinosaur.yaml}
HZ=$(getconf CLK_TCK)
WARM=${WARM:-45}      # TensorRT engine load alone is ~20 s; the counter line
                      # is printed every 100 ticks (10 s at publish_rate 10)

conda deactivate 2>/dev/null; conda deactivate 2>/dev/null
source /opt/ros/humble/setup.bash
source "$IGVC/install/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp AVL_MODELS_DIR=/home/dinosaur/models
export CYCLONEDDS_URI=file://$IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml

cams() {   # awk exits 0 on empty input, so capture first and default to DOWN
  for c in front left right; do
    r=$(timeout 12 ros2 topic hz /zed_$c/zed_node/rgb/color/rect/image 2>&1 |
        grep -m1 'average rate' | awk '{print $3}')
    printf "   zed_%-6s %s Hz\n" "$c" "${r:-DOWN}"
  done
  echo "   load: $(cut -d' ' -f1-3 /proc/loadavg)"
}

# a second costmap node would halve both variants' frames and corrupt the A/B
if pgrep -f '[l]ib/perception_costmap/costmap_node' >/dev/null; then
  echo "ABORT: a costmap_node is already running -- stop it first:"
  pgrep -af '[l]ib/perception_costmap/costmap_node' | cut -c1-100
  exit 1
fi

echo "cameras feeding every run (must not change between variants):"
cams

printf "\n%-9s %-4s %-10s %-7s %-16s %-13s %s\n" \
  variant run costmapHz CPU% "seg(run/cached)" "depth/ipm" "infer(sub/rep/done)"
i=0
for v in $ORDER; do
  i=$((i+1)); LOG=/tmp/ab_${v}_$i.log
  # each variant runs out of its own install tree; nothing else differs
  ( source "$AB_DIR/$v/ros2_ws/install/setup.bash"
    exec ros2 run perception_costmap costmap_node \
         --ros-args --params-file "$CFG" ) >"$LOG" 2>&1 &
  LAUNCHER=$!
  sleep 5
  PID=$(pgrep -P "$LAUNCHER" -f costmap_node | head -1)
  [ -z "$PID" ] && PID=$LAUNCHER
  if ! [ -r /proc/$PID/stat ]; then echo "$v run$i: node did not start, see $LOG"; continue; fi
  sleep $WARM
  A=$(grep -a 'accuracy pipeline' "$LOG" | tail -1)
  [ -z "$A" ] && echo "   ($v run$i: no counter line yet after ${WARM}s -- deltas will read as totals)"
  t0=$(awk '{print $14+$15}' /proc/$PID/stat)
  RATE=$(timeout 14 ros2 topic hz /perception/costmap 2>&1 | grep -m1 'average rate' | awk '{print $3}')
  sleep $((T-14))
  t1=$(awk '{print $14+$15}' /proc/$PID/stat 2>/dev/null || echo "$t0")
  B=$(grep -a 'accuracy pipeline' "$LOG" | tail -1)
  CPU=$(( (t1-t0) * 100 / HZ / T ))
  # every parser defaults to 0: a missing counter must not abort the arithmetic
  d() { local v; v=$(echo "$2" | grep -oE "$1=[0-9]+" | tail -1 | cut -d= -f2); echo "${v:-0}"; }
  f() { local v; v=$(echo "$2" | grep -oE 'frames=[0-9]+/[0-9]+' | tail -1 | cut -d= -f2 | cut -d/ -f$1); echo "${v:-0}"; }
  n() { local v; v=$(echo "$2" | grep -oE 'inference=[0-9]+/[0-9]+/[0-9]+' | tail -1 | cut -d= -f2 | cut -d/ -f$1); echo "${v:-0}"; }
  sub=$(( $(n 1 "$B") - $(n 1 "$A") )); rep=$(( $(n 2 "$B") - $(n 2 "$A") )); don=$(( $(n 3 "$B") - $(n 3 "$A") ))
  dd=$(( $(d depth "$B") - $(d depth "$A") )); pp=$(( $(d ipm_fallback "$B") - $(d ipm_fallback "$A") ))
  if [ "$v" = cache ]; then
    seg="$(( $(f 1 "$B") - $(f 1 "$A") ))/$(( $(f 2 "$B") - $(f 2 "$A") ))"
  else
    seg="n/a"
  fi
  printf "%-9s %-4s %-10s %-7s %-16s %-13s %s\n" \
    "$v" "$i" "${RATE:-?}" "$CPU" "$seg" "$dd/$pp" "$sub/$rep/$don"
  kill -INT "$PID" 2>/dev/null
  for _ in $(seq 12); do [ -d /proc/$PID ] || break; sleep 1; done
  wait "$LAUNCHER" 2>/dev/null
  sleep 3
done
echo
echo "cameras at the end (same as the start = the runs are comparable):"
cams
echo "logs: /tmp/ab_*.log"
