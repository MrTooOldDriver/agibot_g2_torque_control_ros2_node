#!/usr/bin/env bash
# Co-load check with the REAL GDK: the node (C++ and Python) must start, load the GDK next to ROS 2
# without crashing, keep its ROS interface alive while GDK connects, and exit promptly on Ctrl+C.
# The node runs with dry_run:=true, so it never switches the control mode or sends torque, and the
# check is also safe on the robot's own computer, where the GDK really connects. Run it inside the
# container (docker/run.sh: GDK mounted and sourced, workspace sourced):
#   bash src/g2_torque_subscriber/test/coload_check.sh [urdf]
# The URDF defaults to the package's confirmed G2 model, urdf/genie_g2_crs_omnipicker.urdf.
set -u
URDF=${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../urdf" && pwd)/genie_g2_crs_omnipicker.urdf}
PREFIX=$(ros2 pkg prefix g2_torque_subscriber)
fail=0
for rmw in rmw_fastrtps_cpp rmw_cyclonedds_cpp; do
  for impl in cpp py; do
    exe=$PREFIX/lib/g2_torque_subscriber/torque_subscriber_node
    [ "$impl" = py ] && exe=$exe.py
    log=$(mktemp)
    RMW_IMPLEMENTATION=$rmw setsid "$exe" --ros-args -p backend:=gdk -p dry_run:=true \
      -p urdf_path:="$URDF" > "$log" 2>&1 &
    pid=$!
    sleep 6
    alive=no; kill -0 $pid 2> /dev/null && alive=yes
    subs=$(RMW_IMPLEMENTATION=$rmw timeout 20 ros2 topic info --no-daemon \
      /g2_torque_subscriber/left/external_torques 2> /dev/null | awk '/Subscription count/{print $3}')
    t0=$(date +%s%N)
    kill -INT -- -$pid 2> /dev/null
    wait $pid; rc=$?
    dt_ms=$(( ($(date +%s%N) - t0) / 1000000 ))
    crashed=no; grep -qiE "terminate called|Segmentation|bad_array_new_length|core dumped" "$log" && crashed=yes
    ok=yes; { [ "$alive" = yes ] && [ "${subs:-0}" -ge 1 ] && [ "$crashed" = no ]; } || ok=no
    printf '[%s] %-20s %-3s alive_after_6s=%s ros_subscriptions=%s crash=%s exit=%s after %s ms\n' \
      "$([ $ok = yes ] && echo PASS || echo FAIL)" "$rmw" "$impl" "$alive" "${subs:-0}" "$crashed" "$rc" "$dt_ms"
    [ $ok = yes ] || { fail=1; tail -20 "$log"; }
    rm -f "$log"
  done
done
[ $fail = 0 ] && echo "RESULT: PASS" || echo "RESULT: FAIL"
exit $fail
