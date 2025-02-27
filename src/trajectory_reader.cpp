#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TrajectoryReader {
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    std::string file_format_ = "json";  // Default format

public:
    TrajectoryReader() {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/transformed_trajectory_markers", 10);
        
        nh_.getParam("/file_format", file_format_);
        ROS_INFO("Trajectory Reader Node Started. Format: %s", file_format_.c_str());

        readAndPublishTrajectory();
    }

    void readAndPublishTrajectory() {
        std::vector<geometry_msgs::PoseStamped> trajectory;
        
        if (file_format_ == "csv") {
            trajectory = readCSV("trajectory.csv");
        } else if (file_format_ == "yaml") {
            trajectory = readYAML("trajectory.yaml");
        } else {
            trajectory = readJSON("trajectory.json"); // Default
        }

        if (trajectory.empty()) {
            ROS_WARN("No trajectory data found to publish!");
            return;
        }

        publishMarkers(trajectory);
    }

    std::vector<geometry_msgs::PoseStamped> readCSV(const std::string& filename) {
        std::vector<geometry_msgs::PoseStamped> trajectory;
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open CSV file: %s", filename.c_str());
            return trajectory;
        }

        std::string line;
        getline(file, line); // Skip header
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::string timestamp, x, y, z;
            getline(ss, timestamp, ',');
            getline(ss, x, ',');
            getline(ss, y, ',');
            getline(ss, z, ',');

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.pose.position.x = std::stod(x);
            pose.pose.position.y = std::stod(y);
            pose.pose.position.z = std::stod(z);
            trajectory.push_back(pose);
        }
        file.close();
        ROS_INFO("Successfully read CSV trajectory with %lu points.", trajectory.size());
        return trajectory;
    }

    std::vector<geometry_msgs::PoseStamped> readYAML(const std::string& filename) {
        std::vector<geometry_msgs::PoseStamped> trajectory;
        YAML::Node data = YAML::LoadFile(filename);

        if (!data) {
            ROS_ERROR("Failed to open YAML file: %s", filename.c_str());
            return trajectory;
        }

        for (const auto& node : data) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.pose.position.x = node["x"].as<double>();
            pose.pose.position.y = node["y"].as<double>();
            pose.pose.position.z = node["z"].as<double>();
            trajectory.push_back(pose);
        }
        ROS_INFO("Successfully read YAML trajectory with %lu points.", trajectory.size());
        return trajectory;
    }

    std::vector<geometry_msgs::PoseStamped> readJSON(const std::string& filename) {
        std::vector<geometry_msgs::PoseStamped> trajectory;
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open JSON file: %s", filename.c_str());
            return trajectory;
        }

        Json::Value root;
        file >> root;

        for (const auto& node : root) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.pose.position.x = node["x"].asDouble();
            pose.pose.position.y = node["y"].asDouble();
            pose.pose.position.z = node["z"].asDouble();
            trajectory.push_back(pose);
        }
        ROS_INFO("Successfully read JSON trajectory with %lu points.", trajectory.size());
        return trajectory;
    }

    void publishMarkers(const std::vector<geometry_msgs::PoseStamped>& trajectory) {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < trajectory.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "transformed_trajectory";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = trajectory[i].pose.position;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.g = 1.0;  // Green color
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }

        marker_pub_.publish(marker_array);
        ROS_INFO("Published transformed trajectory with %lu markers.", trajectory.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_reader");
    TrajectoryReader node;
    ros::spin();
    return 0;
}
