#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>

class RosbagSynchronizer {
  public:
    RosbagSynchronizer() 
      : nh_("~")
    {
      // Get parameters
      // flags
      nh_.getParam("flag_continue", flag_continue_);
      nh_.getParam("flag_step", flag_step_);
      // topics
      nh_.getParam("color_topic", color_topic_);
      nh_.getParam("color_info", color_info_);
      nh_.getParam("depth_topic", depth_topic_);
      nh_.getParam("depth_info", depth_info_);
      nh_.getParam("accel_topic", accel_topic_);
      nh_.getParam("gyro_topic", gyro_topic_);
      nh_.getParam("imu_topic", imu_topic_);
      nh_.getParam("tf_topic", tf_topic_);
      // fps
      nh_.getParam("color_fps", color_fps_);
      nh_.getParam("depth_fps", depth_fps_);
      // initial timestamp
      double init_timestamp_d;
      nh_.getParam("init_timestamp", init_timestamp_d);
      init_timestamp_ = ros::Time(init_timestamp_d);
      // bag file paths
      nh_.getParam("input_filename", input_filename_);
      nh_.getParam("output_filename", output_filename_);

      // Open bag files
      input_bag_.open(input_filename_, rosbag::bagmode::Read);
      output_bag_.open(output_filename_, rosbag::bagmode::Write);
      topics_.push_back(color_topic_);
      topics_.push_back(color_info_);
      topics_.push_back(depth_topic_);
      topics_.push_back(depth_info_);
      topics_.push_back(accel_topic_);
      topics_.push_back(gyro_topic_);
      topics_.push_back(imu_topic_);
      topics_.push_back(tf_topic_);
    }

    ~RosbagSynchronizer() {
      // Close bag files
      input_bag_.close();
      output_bag_.close();
    }

    void synchronize() {
      ros::Time color_time(0);
      ros::Time depth_time(0);
      bool color_found = false;
      bool depth_found = false;
      ros::Duration color_delta_time = ros::Duration(1 / color_fps_);
      ros::Duration depth_delta_time = ros::Duration(1 / depth_fps_);
      rosbag::View input_view(input_bag_, rosbag::TopicQuery(topics_));

      for (rosbag::MessageInstance const m : input_view) {
        nh_.getParam("flag_continue", flag_continue_);
        while(!flag_continue_ && flag_step_) {
          nh_.getParam("flag_continue", flag_continue_);
          nh_.getParam("flag_step", flag_step_);
          // ROS_INFO("Not able to continue. Flag: %d", flag_continue);
        }
        nh_.setParam("flag_continue", false);

        ROS_INFO_STREAM("topic : " << m.getTopic());
        // color images
        if (m.getTopic() == color_topic_) {
          ROS_DEBUG("Color image");
          adjustTimeStamp(m, "color", color_found, color_time, color_delta_time);
          ROS_DEBUG_STREAM("color_time : " << color_time.toNSec());
        }
        // depth images
        else if (m.getTopic() == depth_topic_) {
          ROS_DEBUG("Depth image");
          adjustTimeStamp(m, "depth", depth_found, depth_time, depth_delta_time);
          ROS_DEBUG_STREAM("depth_time : " << depth_time.toNSec());
        }
        // camera_info, accel, gyro, imu, and tf topics
        else {
          ROS_DEBUG("Not interested packet.");
          output_bag_.write(m.getTopic(), m.getTime(), m);
        }
      }
      
    }

    void adjustTimeStamp(rosbag::MessageInstance const &m, std::string const &image_type, bool &image_found, ros::Time &timestamp, ros::Duration const &delta_time) {
      sensor_msgs::Image::ConstPtr frame = m.instantiate<sensor_msgs::Image>();
      ROS_DEBUG_STREAM("image_found : " << image_found);
      ROS_DEBUG_STREAM("_time : " << timestamp.toNSec());
      if (!image_found) {
        timestamp = init_timestamp_;
        image_found = true;
      }
      else {
        timestamp += delta_time;
        ROS_DEBUG_STREAM("delta_time: " << delta_time.toNSec());
        ROS_DEBUG_STREAM("timestamp += delta_time");
      }
      ROS_INFO_STREAM("Got a " << image_type << "image packet.");
      ROS_INFO_STREAM(image_type << " image timestamp: " << frame->header.stamp.toNSec());
      ROS_INFO_STREAM("New timestamp: " << timestamp.toNSec());

      // new frame with modified timestamp
      sensor_msgs::Image newframe = *frame;
      newframe.header.stamp = timestamp;

      ROS_DEBUG_STREAM("topic timestamp: " << timestamp.toNSec());
      output_bag_.write(m.getTopic(), timestamp, newframe);
    }

  protected:
    // Flags
    bool flag_continue_;
    bool flag_step_;

    // Bag file paths
    std::string input_filename_;
    std::string output_filename_;

    // Topics
    std::string color_topic_;
    std::string color_info_;
    std::string depth_topic_;
    std::string depth_info_;
    std::string accel_topic_;
    std::string gyro_topic_;
    std::string imu_topic_;
    std::string tf_topic_;
    std::vector<std::string> topics_;

    // Fps parameters
    double color_fps_;
    double depth_fps_;

    // Initial timestamp
    ros::Time init_timestamp_;

    // ROS variables
    ros::NodeHandle nh_;
    rosbag::Bag input_bag_;
    rosbag::Bag output_bag_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosbag_synchronizer");
  RosbagSynchronizer rosbag_synchronizer;
  ROS_INFO("node created");
  rosbag_synchronizer.synchronize();
  
  return 0;
}
