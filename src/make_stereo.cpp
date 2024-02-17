#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> my_policy;
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
      nh_.getParam("right_topic", right_topic_);
      nh_.getParam("color_topic", color_topic_);
      nh_.getParam("color_info", color_info_);
      nh_.getParam("depth_topic", depth_topic_);
      nh_.getParam("depth_info", depth_info_);
      nh_.getParam("accel_topic", accel_topic_);
      nh_.getParam("gyro_topic", gyro_topic_);
      nh_.getParam("imu_topic", imu_topic_);
      nh_.getParam("tf_topic", tf_topic_);
      // frame id
      nh_.getParam("right_frame_id", right_frame_id_);
      // focal length
      nh_.getParam("focal_length", focal_length_);
      // baseline
      nh_.getParam("baseline", baseline_);
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

      // Publisher
      right_image_publisher_ = nh_.advertise<sensor_msgs::Image>("/camera/color_right/image_raw", 5);
    }

    ~MakeStereo() {
      // Close bag files
      input_bag_.close();
      output_bag_.close();
    }

    void getRightImage(cv::Mat const &left_image, cv::Mat const &depth_image, cv::Mat &right_image) {
      // ROS_INFO_STREAM("calculating right image ...");
      // ROS_INFO_STREAM("left image type : " << left_image.type());

      short depth = 0;
      int disparity = 0;
      right_image = cv::Mat::zeros(left_image.rows, left_image.cols, left_image.type());

      for (int row = 0; row < left_image.rows; row++) {
          for (int col = 0; col < left_image.cols; col++) {
              depth = depth_image.at<short>(row, col);
              if (depth > 0) {
                  disparity = (int)(focal_length_ * baseline_ / depth);
                  // ROS_DEBUG_STREAM("(row, col): (" << row << ", " << col << ")");
                  // ROS_DEBUG_STREAM("disparity: " << disparity);

                  if (col - disparity > 0) {
                      right_image.at<cv::Vec3b>(row, col - disparity) = left_image.at<cv::Vec3b>(row, col);
                  }
              }
          }
      }

      ROS_INFO_STREAM("right image type : " << right_image.type());
    }

    // rosbag에서 color, depth 이미지를 추출해 
    // right image를 만들어 output bag에 저장한다.
    void makeStereo_bag() {
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
          ROS_DEBUG_STREAM("color seq: " << color_image_ptr->header.seq
                        << "depth seq: " << depth_image_ptr->header.seq);
          // calculate right image
          getRightImage(color_image_ptr->image, depth_image_ptr->image, right_image);
          right_image_ptr = color_image_ptr;
          right_image_ptr->image = right_image;
          right_image_ptr->header.frame_id = "camera_color_right_optical_frame";

          // cv::imshow("left", color_image_ptr->image);
          // cv::imshow("right", right_image);
          // cv::imshow("depth", depth_image_ptr->image);

          // Write to output bag file
          output_bag_.write("/camera/color/image_raw", m.getTime(), color_message_ptr);
          output_bag_.write("/camera/aligned_depth_to_color/image_raw", m.getTime(), depth_message_ptr);
          output_bag_.write("/camera/color_right/image_raw", m.getTime(), right_image_ptr->toImageMsg());
          
          got_color_image = false;
          got_depth_image = false;
        }
      }
      
    }

    void callbackMakeStereo(const sensor_msgs::ImageConstPtr& left_message, const sensor_msgs::ImageConstPtr& depth_message) {
      cv_bridge::CvImagePtr color_image_ptr = cv_bridge::toCvCopy(left_message, sensor_msgs::image_encodings::BGR8);
      cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(depth_message, sensor_msgs::image_encodings::TYPE_16UC1);
      cv::Mat right_image;
      getRightImage(color_image_ptr->image, depth_image_ptr->image, right_image);

      // make right image message
      cv_bridge::CvImagePtr right_image_ptr = color_image_ptr;
      right_image_ptr->image = right_image;
      right_image_ptr->header.frame_id = "camera_color_right_optical_frame";
      // Publish the topic
      right_image_publisher_.publish(right_image_ptr->toImageMsg());
    }

    // topic에서 color, depth 이미지를 subscribe하여
    // right image를 publish한다.
    void makeStereo_topic() {
      message_filters::Subscriber<sensor_msgs::Image> color_image_subscriber(nh_, "/camera/color/image_raw", 1);
      message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber(nh_, "/camera/depth/image_rect_raw", 1);
      message_filters::Synchronizer<my_policy> color_depth_synchronizer(my_policy(10), color_image_subscriber, depth_image_subscriber);
      color_depth_synchronizer.registerCallback(boost::bind(&MakeStereo::callbackMakeStereo, this, _1, _2));
    }
  protected:
    // Flags
    bool flag_continue_;
    bool flag_step_;

    // Bag file paths
    std::string input_filename_;
    std::string output_filename_;

    // Topics
    std::string right_topic_;
    std::string color_topic_;
    std::string color_info_;
    std::string depth_topic_;
    std::string depth_info_;
    std::string accel_topic_;
    std::string gyro_topic_;
    std::string imu_topic_;
    std::string tf_topic_;
    std::vector<std::string> topics_;

    // Frame id
    std::string right_frame_id_;

    // Focal length
    double focal_length_;
    // baseline
    double baseline_;

    // ROS variables
    ros::NodeHandle nh_;
    rosbag::Bag input_bag_;
    rosbag::Bag output_bag_;

    ros::Publisher right_image_publisher_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "make_stereo");
  MakeStereo make_stereo;
  ROS_INFO("node created");
  
  make_stereo.makeStereo_bag();
  
  ros::shutdown();
  return 0;
}
