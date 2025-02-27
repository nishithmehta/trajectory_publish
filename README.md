# Trajectory Publisher and Reader Package

## 📖 Overview
This ROS package provides two nodes to handle robot trajectory visualization and saving:
1. **Trajectory Publisher and Saver Node** (`trajectory_pub`)
   - Collects and stores the robot's trajectory as it moves.
   - Publishes trajectory data as a **MarkerArray** for visualization in RViz.
   - Provides a **ROS service** (`/save_trajectory`) to save trajectory data in **JSON, CSV, or YAML** format.
  
2. **Trajectory Reader and Publisher Node** (`trajectory_reader`)
   - Reads saved trajectory data from a file.
   - Transforms the trajectory to the **odom** frame.
   - Publishes the transformed trajectory for visualization in RViz.

---

## 🛠 Installation

### 1️⃣ **Clone the Repository**
```bash
cd ~/catkin_ws/src
git clone <repository_link> trajectory_publish
```

### 2️⃣ **Build the Package**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3️⃣ **Dependencies**
Make sure you have the following installed:
```bash
sudo apt-get install ros-$(rosversion -d)-visualization-msgs
sudo apt-get install ros-$(rosversion -d)-geometry-msgs
sudo apt-get install ros-$(rosversion -d)-std-srvs
```

---

## Running the Package

### **1️⃣ Start `roscore`**
```bash
roscore
```

### **2️⃣ Start the Trajectory Publisher**
```bash
rosrun trajectory_publish trajectory_pub
```

### **3️⃣ Publish Robot Poses for Testing**
```bash
rostopic pub -1 /robot_pose geometry_msgs/PoseStamped "{
    header: {stamp: now, frame_id: 'map'},
    pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

### **4️⃣ Save the Trajectory**
```bash
rosparam set /save_duration 5   # Save only last 5 seconds of trajectory
rosparam set /save_format "json"  # Choose between json, csv, or yaml
rosservice call /save_trajectory "{}"
```

### **5️⃣ Run the Trajectory Reader**
```bash
rosrun trajectory_publish trajectory_reader
```

---

## Automated Testing Script
Run the full pipeline using the test script:
```bash
cd ~/catkin_ws/src/trajectory_publish
chmod +x test_full_pipeline.sh
./test_full_pipeline.sh
```

---

## Topics & Services

### **📡 Published Topics**
| Topic Name            | Message Type                          | Description                           |
|----------------------|--------------------------------|-----------------------------------|
| `/robot_pose`       | `geometry_msgs/PoseStamped`  | Receives the robot's pose        |
| `/trajectory_markers` | `visualization_msgs/MarkerArray` | Visualizes the trajectory       |

### **📩 Services**
| Service Name          | Service Type       | Description                               |
|----------------------|----------------|-------------------------------------|
| `/save_trajectory`   | `std_srvs/Trigger` | Saves trajectory in a file         |

---

## 🛠 Debugging & Logs
- If a trajectory does not save, check:
  ```bash
  rosparam get /save_format
  ```
- Ensure the topic `/robot_pose` is publishing correctly:
  ```bash
  rostopic echo /robot_pose
  ```
- Check for any errors in the ROS logs:
  ```bash
  roslaunch trajectory_publish trajectory_pub.launch
  ```

---

## 🤝 Contributors
- **Your Name** - Developer  
- **Other Contributors** (if any)  

---

## 🏆 Future Improvements
- Add **real-time playback** of saved trajectories.
- Support **additional trajectory file formats**.
- Integrate with **SLAM algorithms**.

---

## 🏁 License
This package is open-source under the MIT License.

