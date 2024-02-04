#include <stdint.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"

class UniteIMU {
  public:
    UniteIMU() 
            : accel_ready_(false), 
              gyro_ready_(false) {
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("/camera/imu", 100);
        accel_subscriber_ = nh_.subscribe("/camera/accel/sample",
                                        100, 
                                        &UniteIMU::accelCallback, 
                                        this);
        gyro_subscriber_ = nh_.subscribe("/camera/gyro/sample",
                                        100, 
                                        &UniteIMU::gyroCallback, 
                                        this);
    };

    void accelCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gyroCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void publishImu();

  protected:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Subscriber accel_subscriber_;
    ros::Subscriber gyro_subscriber_;
    
    sensor_msgs::Imu imu_msg_;
    std_msgs::Header accel_header_;
    std_msgs::Header gyro_header_;

    bool accel_ready_;
    bool gyro_ready_;
};

// activate when accel sample data comes in.
void UniteIMU::accelCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // copy header
    imu_msg_.header = msg->header;
    accel_header_ = msg->header;

    // copy linear_acceleration data
    imu_msg_.linear_acceleration = msg->linear_acceleration;
    std::copy(std::begin(msg->linear_acceleration_covariance), 
              std::end(msg->linear_acceleration_covariance), 
              std::begin(imu_msg_.linear_acceleration_covariance));

    accel_ready_ = true;
    publishImu();
}

// activate when gyro sample data comes in.
void UniteIMU::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // copy header
    imu_msg_.header = msg->header;
    gyro_header_ = msg->header;

    // copy angular_velocity data
    imu_msg_.angular_velocity = msg->angular_velocity;
    std::copy(std::begin(msg->angular_velocity_covariance), 
              std::end(msg->angular_velocity_covariance), 
              std::begin(imu_msg_.angular_velocity_covariance));

    // copy orientation data
    imu_msg_.orientation = msg->orientation;
    std::copy(std::begin(msg->orientation_covariance), 
              std::end(msg->orientation_covariance), 
              std::begin(imu_msg_.orientation_covariance));

    gyro_ready_ = true;
    publishImu();
}

void UniteIMU::publishImu() {
    if (accel_ready_ && gyro_ready_) {
        imu_msg_.header = (accel_header_.stamp > gyro_header_.stamp) ? 
                            accel_header_ : gyro_header_;
        imu_msg_.header.frame_id = "camera_imu_frame";
        
        imu_publisher_.publish(imu_msg_);
        ROS_INFO("united imu data published");
        // ROS_INFO("header_seq: %d", imu_msg_.header.seq);
        // ROS_INFO("accel_seq: %d", accel_seq_);
        // ROS_INFO("gyro_seq: %d", gyro_seq_);

        accel_ready_ = false;
        gyro_ready_ = false;
    }
    else
        return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unite_imu");
    UniteIMU unite_imu_node;
    ros::spin();

    return 0;
}