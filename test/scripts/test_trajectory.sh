#!/bin/bash

echo "🚀 Starting full trajectory pipeline test..."

# 1️⃣ Ensure ROS is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "⚡ Starting roscore..."
    roscore & 
    sleep 3
fi

# 2️⃣ Launch the Trajectory Publisher node
echo "📌 Starting trajectory publisher node..."
rosrun trajectory_publish trajectory_pub & 
PUBLISHER_PID=$!
sleep 3

# 3️⃣ Publish multiple robot poses at different positions
echo "🤖 Publishing robot poses..."
for i in {1..5}; do
    X=$(printf "%.2f" "$(echo "$i * 1.5" | bc)")   
    Y=$(printf "%.2f" "$(echo "$i * -0.5" | bc)")  
    Z=0.0
    echo "📍 Publishing Pose: X=$X, Y=$Y, Z=$Z"
    
    rostopic pub -1 /robot_pose geometry_msgs/PoseStamped "{
        header: {stamp: now, frame_id: 'map'},
        pose: {position: {x: $X, y: $Y, z: $Z}, orientation: {w: 1.0}}
    }"
    
    sleep 1  
done

# 4️⃣ Set parameters for saving
echo "⚙️ Setting parameters..."
rosparam set /save_duration 5  
rosparam set /save_format "json"  

# 5️⃣ Call the service to save trajectory
echo "💾 Calling /save_trajectory service..."
rosservice call /save_trajectory "{}"
sleep 2

# 6️⃣ Check saved trajectory file
echo "📂 Checking saved trajectory file..."
if [ -f trajectory.json ]; then
    echo "✅ JSON Trajectory Data:"
    cat trajectory.json
elif [ -f trajectory.csv ]; then
    echo "✅ CSV Trajectory Data:"
    cat trajectory.csv
elif [ -f trajectory.yaml ]; then
    echo "✅ YAML Trajectory Data:"
    cat trajectory.yaml
else
    echo "❌ No trajectory file found!"
fi

# 7️⃣ Kill the Trajectory Publisher
echo "🛑 Stopping Trajectory Publisher..."
kill $PUBLISHER_PID

# 8️⃣ Start the Trajectory Reader Node
echo "📌 Starting Trajectory Reader Node..."
rosrun trajectory_publish trajectory_reader &
READER_PID=$!
sleep 3  

echo "✅ Trajectory Reader is now visualizing the saved trajectory..."

# 9️⃣ Wait and stop the Reader node
sleep 5
echo "🛑 Stopping Trajectory Reader..."
kill $READER_PID

echo "🎉 Full test pipeline complete!"
