#!/bin/bash
# Launch Pupper Realtime Voice System

echo "🚀 Starting Pupper Realtime System"
echo ""

# Trap Ctrl+C to kill all background processes
trap 'kill $(jobs -p) 2>/dev/null; exit' INT TERM

echo "Starting processes..."
echo ""

# Launch ROS2
echo "1. Launching ROS2..."
ros2 launch launch/launch.py > /tmp/ros2_launch.log 2>&1 &
ROS2_PID=$!

# Wait a moment for ROS2 to initialize
sleep 2

# Launch Realtime Voice Node
echo "2. Launching Realtime Voice..."
python3 pupper_llm/realtime_voice.py > /tmp/realtime_voice.log 2>&1 &
VOICE_PID=$!

# Launch Karel Commander
echo "3. Launching Karel Commander..."
python3 pupper_llm/karel/karel_realtime_commander.py > /tmp/karel_commander.log 2>&1 &
KAREL_PID=$!

echo ""
echo "✅ All processes started!"
echo ""
echo "Logs:"
echo "  - ROS2: /tmp/ros2_launch.log"
echo "  - Voice: /tmp/realtime_voice.log"
echo "  - Karel: /tmp/karel_commander.log"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for all background processes
wait

