#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosbag_synchronizer");
  ros::NodeHandle nh;
  bool flag_continue;
  nh.getParam("flag_continue", flag_continue);
  ROS_INFO("Continue falg: %d", flag_continue);
  
  std::string input_filename = "/home/kodogyu/Datasets/rosbags/l515_record_2024-02-01-15-23-23.bag";
  std::string output_filename = "/home/kodogyu/Datasets/rosbags/l515_synchronized.bag";

  rosbag::Bag input_bag(input_filename, rosbag::bagmode::Read);
  rosbag::Bag output_bag(output_filename, rosbag::bagmode::Write);

  std::string c_topic = "/camera/color/image_raw";
  std::string c_info = "/camera/color/camera_info";
  std::string d_topic = "/camera/depth/image_rect_raw";
  std::string d_info = "/camera/depth/camera_info";
  std::string accel_topic = "/camera/accel/sample";
  std::string gyro_topic = "/camera/gyro/sample";
  std::string tf_topic = "/tf_static";

  std::vector<std::string> topics;
  topics.push_back(c_topic);
  topics.push_back(c_info);
  topics.push_back(d_topic);
  topics.push_back(d_info);
  topics.push_back(accel_topic);
  topics.push_back(gyro_topic);
  topics.push_back(tf_topic);

  rosbag::View input_view(input_bag, rosbag::TopicQuery(topics));

  ros::Time color_time;
  ros::Time depth_time;
  bool color_found = false;
  bool depth_found = false;
  ros::Time init_time(1706768602.954694000);
  ros::Duration color_delta_t = ros::Duration(0, 66400000);
  ros::Duration depth_delta_t = ros::Duration(0, 33200000);

  // Iterate through the messages on the sync_topic
  for (rosbag::MessageInstance const m : input_view) {
    while(!flag_continue) {
      nh.getParam("flag_continue", flag_continue);
        ROS_INFO("Not able to continue. Flag: %d", flag_continue);
    }

    // color images
    if (m.getTopic() == c_topic) {
      if (!color_found) {
        color_time = init_time;
        color_found = true;
      }
      else {
        color_time += color_delta_t;
      }
      ROS_INFO("Got a color image packet.");
      ROS_INFO("Color image timestamp: %f", m.getTime().toSec());
      ROS_INFO("New timestamp: %f", color_time.toSec());
      output_bag.write(m.getTopic(), color_time, m);
    }
    // depth images
    else if (m.getTopic() == d_topic) {
      if (!depth_found) {
        depth_time = init_time;
        depth_found = true;
      }
      else {
        depth_time += depth_delta_t;
      }
      ROS_INFO("Got a depth image packet.");
      ROS_INFO("Depth image timestamp: %f", m.getTime().toSec());
      ROS_INFO("New timestamp: %f", depth_time.toSec());
      output_bag.write(m.getTopic(), depth_time, m);
    }
    // camera_info, accel, gyro, tf topics
    else {
      output_bag.write(m.getTopic(), m.getTime(), m);
    }
    nh.setParam("flag_continue", false);
  }

  input_bag.close();
  output_bag.close();

  return 0;
}
