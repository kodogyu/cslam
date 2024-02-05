#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>

class RosbagSynchronizer {
  public:
    RosbagSynchronizer() {
      nh.getParam("flag_continue", flag_continue);
      ROS_INFO_STREAM("Continue flag: " << flag_continue);
      nh.getParam("flag_step", flag_step);
      ROS_INFO_STREAM("Debug flag: " << flag_continue);

      input_bag_.open(input_filename, rosbag::bagmode::Read);
      output_bag_.open(output_filename, rosbag::bagmode::Write);

      topics.push_back(c_topic);
      topics.push_back(c_info);
      topics.push_back(d_topic);
      topics.push_back(d_info);
      topics.push_back(accel_topic);
      topics.push_back(gyro_topic);
      topics.push_back(tf_topic);
    }

    ~RosbagSynchronizer() {
      input_bag_.close();
      output_bag_.close();
    }

    void synchronize() {
      ros::Time color_time;
      ros::Time depth_time;
      bool color_found = false;
      bool depth_found = false;
      ros::Time init_time(1706768602.954694000);
      ros::Duration color_delta_t = ros::Duration(0, 66400000);
      ros::Duration depth_delta_t = ros::Duration(0, 33200000);
      
      rosbag::View input_view(input_bag_, rosbag::TopicQuery(topics));

      for (rosbag::MessageInstance const m : input_view) {
        nh.getParam("flag_continue", flag_continue);
        nh.getParam("flag_step", flag_step);
        while(!flag_continue && flag_step) {
          nh.getParam("flag_continue", flag_continue);
          nh.getParam("flag_step", flag_step);
          // ROS_INFO("Not able to continue. Flag: %d", flag_continue);
        }
        nh.setParam("flag_continue", false);

        // color images
        if (m.getTopic() == c_topic) {
          sensor_msgs::Image::ConstPtr frame = m.instantiate<sensor_msgs::Image>();
          if (!color_found) {
            color_time = init_time;
            color_found = true;
          }
          else {
            color_time += color_delta_t;
          }
          ROS_INFO("Got a color image packet.");
          ROS_INFO("Color image timestamp: %f", frame->header.stamp.toSec());
          ROS_INFO("New timestamp: %f", color_time.toSec());

          // new frame with modified timestamp
          sensor_msgs::Image newframe = *frame;
          newframe.header.stamp = color_time;

          ROS_INFO("topic timestamp: %f", depth_time.toSec());
          output_bag_.write(m.getTopic(), color_time, newframe);
        }
        // depth images
        else if (m.getTopic() == d_topic) {
          sensor_msgs::Image::ConstPtr frame = m.instantiate<sensor_msgs::Image>();
          if (!depth_found) {
            depth_time = init_time;
            depth_found = true;
          }
          else {
            depth_time += depth_delta_t;
          }
          ROS_INFO("Got a depth image packet.");
          ROS_INFO("Depth image timestamp: %f", frame->header.stamp.toSec());
          ROS_INFO("New timestamp: %f", depth_time.toSec());

          // new frame with modified timestamp
          sensor_msgs::Image newframe = *frame;
          newframe.header.stamp = depth_time;

          ROS_INFO("topic timestamp: %f", depth_time.toSec());
          output_bag_.write(m.getTopic(), depth_time, newframe);
        }
        // camera_info, accel, gyro, tf topics
        else {
          ROS_INFO("Not interested packet.");
          output_bag_.write(m.getTopic(), m.getTime(), m);
        }
      }
      
    }

  protected:
    ros::NodeHandle nh;
    bool flag_continue;
    bool flag_step;

    rosbag::Bag input_bag_;
    rosbag::Bag output_bag_;

    std::string input_filename = "/home/kodogyu/Datasets/rosbags/l515_record_2024-02-01-15-23-23.bag";
    std::string output_filename = "/home/kodogyu/Datasets/rosbags/l515_synchronized.bag";

    std::string c_topic = "/camera/color/image_raw";
    std::string c_info = "/camera/color/camera_info";
    std::string d_topic = "/camera/depth/image_rect_raw";
    std::string d_info = "/camera/depth/camera_info";
    std::string accel_topic = "/camera/accel/sample";
    std::string gyro_topic = "/camera/gyro/sample";
    std::string tf_topic = "/tf_static";
    std::vector<std::string> topics;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosbag_synchronizer");
  RosbagSynchronizer rosbag_synchronizer;
  ROS_INFO("node created");
  rosbag_synchronizer.synchronize();
  return 0;
}
