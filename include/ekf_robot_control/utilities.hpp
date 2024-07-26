#ifndef INCLUDE_UTILITIES_HPP_
#define INCLUDE_UTILITIES_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include "extendedKalmanFilter.hpp"

class Utilities
{
    public:
        Utilities();
        ~Utilities();

        void getOdometryMeasurement(const nav_msgs::msg::Odometry &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, double twoDimensionalMode);
        void getAccelerationMeasurement(const sensor_msgs::msg::Imu &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, double twoDimensionalMode);

    private:
        double gravitationalAcceleration_ = 9.80665; // m/s^2
};

#endif  //  INCLUDE_UTILITIES_HPP_
