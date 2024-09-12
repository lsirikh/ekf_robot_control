#include "ekf_robot_control/setMeasurementCovariance.hpp"

SetMeasurementCovariance::SetMeasurementCovariance(bool imu) : Node("set_measurement_covariance"), imu_(imu) {
  imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 10, std::bind(&SetMeasurementCovariance::imuCallback, this, std::placeholders::_1));
  imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SetMeasurementCovariance::odomCallback, this, std::placeholders::_1));
  odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/data", 10);
}

SetMeasurementCovariance::~SetMeasurementCovariance(){
}

void SetMeasurementCovariance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  odomData_ = *msg;
  //odomData_.header.frame_id = "map";
  odomData_.header.frame_id = "odom";
  odomData_.pose.covariance[0] = 1e-3;     // x
  odomData_.pose.covariance[7] = 1e-3;     // y
  odomData_.pose.covariance[14] = 1e-300;  // z
  odomData_.pose.covariance[21] = 1e-3;    // roll
  odomData_.pose.covariance[28] = 1e-3;    // pitch
  odomData_.pose.covariance[35] = 1e-3;    // yaw
  odomData_.twist.covariance[0] = 1e-3;    // vx
  odomData_.twist.covariance[7] = 1e-300;  // vy
  odomData_.twist.covariance[14] = 1e-3;   // vz
  odomData_.twist.covariance[21] = 1e-3;   // omegax
  odomData_.twist.covariance[28] = 1e-3;   // omegay
  odomData_.twist.covariance[35] = 1e-3;   // omegaz
  odomPub_->publish(odomData_);
}

void SetMeasurementCovariance::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
  imuData_ = *msg;
  imuData_.orientation_covariance[0] = 1e-2;           // roll
  imuData_.orientation_covariance[4] = 1e-2;           // pitch
  imuData_.orientation_covariance[8] = 1e-2;           // yaw
  imuData_.linear_acceleration_covariance[0] = 1e-2;   // ax
  imuData_.linear_acceleration_covariance[4] = 1e-2;   // ay
  imuData_.linear_acceleration_covariance[8] = 1e-2;   // az
  imuData_.angular_velocity_covariance[0] = 1e-2;      // omegax
  imuData_.angular_velocity_covariance[4] = 1e-2;      // omegay
  imuData_.angular_velocity_covariance[8] = 1e-2;      // omegaz
  imuPub_->publish(imuData_);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetMeasurementCovariance>(true);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
