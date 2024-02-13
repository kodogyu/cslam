#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class MakeStereo {
  public:
    MakeStereo() 
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

    ~MakeStereo() {
      // Close bag files
      input_bag_.close();
      output_bag_.close();
    }

    void makeStereo() {
      bool got_color_image = false;
      bool got_depth_image = false;
      sensor_msgs::ImageConstPtr color_message_ptr;
      sensor_msgs::ImageConstPtr depth_message_ptr;
      cv_bridge::CvImagePtr right_image_ptr;
      cv_bridge::CvImagePtr color_image_ptr;
      cv_bridge::CvImagePtr depth_image_ptr;
      cv::Mat right_image;
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
          color_message_ptr = m.instantiate<sensor_msgs::Image>();
          color_image_ptr = cv_bridge::toCvCopy(color_message_ptr, sensor_msgs::image_encodings::BGR8);
          got_color_image = true;
        }
        // depth images
        else if (m.getTopic() == depth_topic_) {
          ROS_DEBUG("Depth image");
          depth_message_ptr = m.instantiate<sensor_msgs::Image>();
          depth_image_ptr = cv_bridge::toCvCopy(depth_message_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
          got_depth_image = true;
        }
        // etc. (camera_info, accel, gyro, imu, ...)
        else {
          ROS_DEBUG("Not interested packet.");
          output_bag_.write(m.getTopic(), m.getTime(), m);
          continue;
        }

        // both frames are received
        if (got_color_image && got_depth_image) {
          // calculate right image
          getRightImage(color_image_ptr->image, depth_image_ptr->image, right_image);
          right_image_ptr = color_image_ptr;
          right_image_ptr->image = right_image;

          // cv::imshow("left", color_image_ptr->image);
          // cv::imshow("right", right_image);
          // cv::imshow("depth", depth_image_ptr->image);

          // Write to output bag file
          output_bag_.write("/camera/color/image_raw", m.getTime(), color_message_ptr);
          output_bag_.write("/camera/depth/image_rect_raw", m.getTime(), depth_message_ptr);
          output_bag_.write("/camera/color_right/image_raw", m.getTime(), right_image_ptr->toImageMsg());
          
          got_color_image = false;
          got_depth_image = false;
        }
      }
      
    }

    void getRightImage(cv::Mat const &left_image, cv::Mat const &depth_image, cv::Mat &right_image) {
      // ROS_INFO_STREAM("calculating right image ...");
      // ROS_INFO_STREAM("left image type : " << left_image.type());

      int focal_length = 604;  // f_x = 603.838, f_y = 604.019
      int baseline = 10;  // 10mm
      short depth = 0;
      int disparity = 0;
      right_image = cv::Mat::zeros(left_image.rows, left_image.cols, left_image.type());

      for (int row = 0; row < left_image.rows; row++) {
          for (int col = 0; col < left_image.cols; col++) {
              depth = depth_image.at<short>(row, col);
              if (depth > 0) {
                  disparity = focal_length * baseline / depth;
                  ROS_DEBUG_STREAM("(row, col): (" << row << ", " << col << ")");
                  ROS_DEBUG_STREAM("disparity: " << disparity);

                  if (col - disparity > 0) {
                      right_image.at<cv::Vec3b>(row, col - disparity) = left_image.at<cv::Vec3b>(row, col);
                  }
              }
          }
      }

      ROS_INFO_STREAM("right image type : " << right_image.type());
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

    // ROS variables
    ros::NodeHandle nh_;
    rosbag::Bag input_bag_;
    rosbag::Bag output_bag_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "make_stereo");
  MakeStereo make_stereo;
  ROS_INFO("node created");
  
  make_stereo.makeStereo();
  
  ros::shutdown();
  return 0;
}
