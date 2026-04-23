#!/bin/bash
# Runs nav2_sim.launch.py, captures all output and ROS2 per-process logs
# into a single timestamped report file.

set -eo pipefail

WORKSPACE=~/REAL_RDT
REPORT_DIR=$WORKSPACE/logs
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
REPORT=$REPORT_DIR/launch_report_$TIMESTAMP.txt

mkdir -p "$REPORT_DIR"

echo "=== ROS2 nav2_sim Launch Report ===" > "$REPORT"
echo "Started: $TIMESTAMP" >> "$REPORT"
echo "Host:    $(hostname)" >> "$REPORT"
echo "" >> "$REPORT"

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

# Record the log dir state before launch so we can find the new one after
BEFORE=$(ls ~/.ros/log/ | grep -v latest)

echo "=== LAUNCH OUTPUT ===" >> "$REPORT"
echo "" >> "$REPORT"

# Run the launch and tee stdout+stderr into the report
set +e
ros2 launch nyurdt_navigation nav2_sim.launch.py 2>&1 | tee -a "$REPORT"
LAUNCH_EXIT=${PIPESTATUS[0]}
set -e

echo "" >> "$REPORT"
echo "=== LAUNCH EXITED (code: $LAUNCH_EXIT) ===" >> "$REPORT"
echo "" >> "$REPORT"

# Find the new ROS2 log directory created during this run
AFTER=$(ls ~/.ros/log/ | grep -v latest)
NEW_DIR=$(comm -13 <(echo "$BEFORE" | sort) <(echo "$AFTER" | sort) | grep -v '\.log$' | head -1)

if [ -n "$NEW_DIR" ]; then
    ROS_LOG_DIR=~/.ros/log/$NEW_DIR
    echo "=== ROS2 LOG DIRECTORY: $NEW_DIR ===" >> "$REPORT"
    echo "" >> "$REPORT"

    # Append the main launch.log
    if [ -f "$ROS_LOG_DIR/launch.log" ]; then
        echo "--- launch.log ---" >> "$REPORT"
        cat "$ROS_LOG_DIR/launch.log" >> "$REPORT"
        echo "" >> "$REPORT"
    fi

    # Append every per-process log file from the flat log dir
    for LOG in ~/.ros/log/*.log; do
        # Only include logs newer than the run start
        if [ "$LOG" -nt "$REPORT" ] 2>/dev/null || find "$LOG" -newer "$REPORT" -print 2>/dev/null | grep -q .; then
            echo "--- $(basename $LOG) ---" >> "$REPORT"
            cat "$LOG" >> "$REPORT"
            echo "" >> "$REPORT"
        fi
    done
fi

echo "=== END OF REPORT ===" >> "$REPORT"
echo ""
echo "Report saved to: $REPORT"
