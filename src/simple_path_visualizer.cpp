#include <vector>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class PathVisualizer {
public:
    PathVisualizer()
      : nh_("~") {
        path_publisher_ = nh_.advertise<nav_msgs::Path>("/path_for_rviz", 10);
        pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
        
        // Load csv file
        nh_.getParam("filename", filename_);
        std::ifstream file(filename_);

        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
        }

        std::string line;
        std::getline(file, line);  // skip first line
        while (std::getline(file, line)) {
            std::vector<std::string> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            data_.push_back(row);
        }
        data_length_ = data_.size();
        ROS_INFO("Data reading done.");
    }

    void publishPath() {
        double timestamp, x, y, z, qw, qx, qy, qz, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz;
        parseData(timestamp, x, y, z, qw, qx, qy, qz, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz);

        // Create a nav_msgs/Path message
        path_msg_.header.frame_id = "map";

        // Generate some dummy path points
        geometry_msgs::PoseStamped pose_stamped;
        // header
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = ros::Time::now();
        // position
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = z;
        // orientation
        pose_stamped.pose.orientation.w = qw;
        pose_stamped.pose.orientation.x = qx;
        pose_stamped.pose.orientation.y = qy;
        pose_stamped.pose.orientation.z = qz;
        path_msg_.poses.push_back(pose_stamped);

        // Publish the path message
        ROS_INFO_STREAM("iter [" << iter_ << "] publishing path point (" << x << ", " << y << ", " << z << ")");
        path_publisher_.publish(path_msg_);
        // publish the pose message
        pose_publisher_.publish(pose_stamped);

        iter_++;
    }

    void parseData(double &timestamp, double &x, double &y, double &z,
                    double &qw, double &qx, double &qy, double &qz,
                    double &vx, double &vy, double &vz,
                    double &bgx, double &bgy, double &bgz,
                    double &bax, double &bay, double &baz) {
        int iter = iter_ % data_length_;

        timestamp = std::stod(data_[iter][0]);
        x = std::stod(data_[iter][1]);
        y = std::stod(data_[iter][2]);
        z = std::stod(data_[iter][3]);
        qw = std::stod(data_[iter][4]);
        qx = std::stod(data_[iter][5]);
        qy = std::stod(data_[iter][6]);
        qz = std::stod(data_[iter][7]);
        vx = std::stod(data_[iter][8]);
        vy = std::stod(data_[iter][9]);
        vz = std::stod(data_[iter][10]);
        bgx = std::stod(data_[iter][11]);
        bgy = std::stod(data_[iter][12]);
        bgz = std::stod(data_[iter][13]);
        bax = std::stod(data_[iter][14]);
        bay = std::stod(data_[iter][15]);
        baz = std::stod(data_[iter][16]);
    }

public:
    ros::NodeHandle nh_;
    ros::Publisher path_publisher_;
    ros::Publisher pose_publisher_;
    nav_msgs::Path path_msg_;

    std::string filename_;
    std::vector<std::vector<std::string>> data_;
    int data_length_;

    int iter_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_visualizer");
    PathVisualizer path_visualizer;

    // Publish path
    float frequency = 3;
    ros::Rate ros_rate = ros::Rate(frequency);
    while (ros::ok()) {
        path_visualizer.publishPath();

        if (path_visualizer.nh_.getParam("frequency", frequency)) {
            ros_rate = ros::Rate(frequency);
        }
        ros_rate.sleep();
    }

    // Shutdown ROS
    ros::shutdown();
    return 0;
}
