#ifndef INCLUDE_SETMEASUREMENTCOVARIANCE_HPP_
#define INCLUDE_SETMEASUREMENTCOVARIANCE_HPP_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class SetMeasurementCovariance : public rclcpp::Node
{
    public:
        SetMeasurementCovariance(bool imu);
        ~SetMeasurementCovariance();

        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    private:
        double imu_;
        sensor_msgs::msg::Imu imuData_;
        nav_msgs::msg::Odometry odomData_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
};

#endif  //  INCLUDE_SETMEASUREMENTCOVARIANCE_HPP_
