#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>  // Ensure you install yaml-cpp for YAML support

class TrajectoryNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer save_service_;

    std::vector<std::pair<double, geometry_msgs::PoseStamped>> trajectory_data_;
    std::string save_format_ = "json";  // Default format

public:
    TrajectoryNode() {
        ROS_INFO("[TrajectoryNode] Initializing...");

        pose_sub_ = nh_.subscribe("/robot_pose", 10, &TrajectoryNode::poseCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 10);
        save_service_ = nh_.advertiseService("/save_trajectory", &TrajectoryNode::saveTrajectory, this);

        ROS_INFO("[TrajectoryNode] Node started successfully!");
        ROS_INFO("[TrajectoryNode] Listening on topic: /robot_pose");
        ROS_INFO("[TrajectoryNode] Publishing markers on topic: /trajectory_markers");
        ROS_INFO("[TrajectoryNode] Service available: /save_trajectory");
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double timestamp = ros::Time::now().toSec();
        trajectory_data_.push_back({timestamp, *msg});

        ROS_DEBUG("[TrajectoryNode] Received new pose at time %.3f: (x=%.2f, y=%.2f, z=%.2f)",
                  timestamp, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        publishMarkers();
    }

    void publishMarkers() {
        if (trajectory_data_.empty()) {
            ROS_WARN("[TrajectoryNode] No trajectory data available. Skipping marker publishing.");
            return;
        }

        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < trajectory_data_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectory";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = trajectory_data_[i].second.pose.position;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
        
        marker_pub_.publish(marker_array);
        ROS_INFO("[TrajectoryNode] Published %ld markers to /trajectory_markers", trajectory_data_.size());
    }

    bool saveTrajectory(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        double duration = 10.0;  // Default to last 10 seconds
        nh_.getParam("/save_duration", duration);
        nh_.getParam("/save_format", save_format_);

        ROS_INFO("[TrajectoryNode] Saving trajectory with duration: %.1f seconds, Format: %s", duration, save_format_.c_str());

        double current_time = ros::Time::now().toSec();
        std::vector<std::pair<double, geometry_msgs::PoseStamped>> filtered_trajectory;

        for (auto it = trajectory_data_.rbegin(); it != trajectory_data_.rend(); ++it) {
            if (current_time - it->first <= duration) {
                filtered_trajectory.push_back(*it);
            } else {
                break;
            }
        }

        if (filtered_trajectory.empty()) {
            ROS_WARN("[TrajectoryNode] No trajectory data available for the specified duration.");
            res.success = false;
            res.message = "No trajectory data available for the specified duration.";
            return true;
        }

        ROS_INFO("[TrajectoryNode] Filtering complete. Saving %ld trajectory points.", filtered_trajectory.size());

        std::ofstream file;
        std::string filename;

        if (save_format_ == "csv") {
            filename = "trajectory.csv";
            file.open(filename);
            file << "timestamp,x,y,z\n";
            for (const auto& [timestamp, pose] : filtered_trajectory) {
                file << timestamp << "," << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << "\n";
            }
        } else if (save_format_ == "yaml") {
            filename = "trajectory.yaml";
            YAML::Emitter out;
            out << YAML::BeginSeq;
            for (const auto& [timestamp, pose] : filtered_trajectory) {
                out << YAML::BeginMap;
                out << YAML::Key << "timestamp" << YAML::Value << timestamp;
                out << YAML::Key << "x" << YAML::Value << pose.pose.position.x;
                out << YAML::Key << "y" << YAML::Value << pose.pose.position.y;
                out << YAML::Key << "z" << YAML::Value << pose.pose.position.z;
                out << YAML::EndMap;
            }
            out << YAML::EndSeq;
            file.open(filename);
            file << out.c_str();
        } else { // Default to JSON
            filename = "trajectory.json";
            file.open(filename);
            file << "[\n";
            for (size_t i = 0; i < filtered_trajectory.size(); ++i) {
                const auto& [timestamp, pose] = filtered_trajectory[i];
                file << "  { \"timestamp\": " << timestamp
                     << ", \"x\": " << pose.pose.position.x
                     << ", \"y\": " << pose.pose.position.y
                     << ", \"z\": " << pose.pose.position.z << " }";
                if (i < filtered_trajectory.size() - 1) file << ",";
                file << "\n";
            }
            file << "]";
        }

        file.close();
        ROS_INFO("[TrajectoryNode] Trajectory saved successfully to %s", filename.c_str());

        res.success = true;
        res.message = "Trajectory saved successfully in " + save_format_ + " format.";
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ROS_INFO("[TrajectoryNode] Starting Trajectory Publisher and Saver Node...");
    TrajectoryNode node;
    ros::spin();
    return 0;
}
