#ifndef INCLUDE_INTERFACE_HPP_
#define INCLUDE_INTERFACE_HPP_

#include "extendedKalmanFilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 추가된 헤더
#include "measurement.hpp"
#include "utilities.hpp"
#include <Eigen/Dense>

class Interface : public rclcpp::Node
{
    public:
        Interface(bool imu, bool odometry);
        ~Interface();

        void storeMeasurement(Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, std::vector<int> data);
        void fuseData();
        void publishOdometry(Eigen::VectorXd states);
        void publishTransform(Eigen::VectorXd states);
        void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
        void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    private:
        ExtendedKalmanFilter ekf_;
        Utilities data_;
        bool useIMU_;
        bool useOdometry_;
        bool visualizeModel_ = true;
        int queueIMU_ = 1;
        int queueOdometry_ = 1;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        int stateSize_ = 15;
        std::vector<Measurement> sensorMeasurements_;
        Measurement sensor_;
        std::vector<int> dataIMU = 
        {
            0, 0, 0, // x y z
            0, 0, 0, // r p y 
            0, 0, 0, // vx vy vz
            0, 0, 1, // vx vy vz
            1, 1, 1  // ax ay az
        };  // x y z r p y vx vy vz vr vp vy ax ay az
        std::vector<int> dataOdometry = 
        {
            1, 1, 0, // x y z
            0, 0, 1, // r p y 
            0, 0, 0, // vx vy vz
            0, 0, 0, // vr vp vy
            0, 0, 0  // ax ay az
        }; // x y z r p y vx vy vz vr vp vy ax ay az
        double twoDimensionalMode = true;
        bool firstMeasurement = false;
};

#endif  //  INCLUDE_INTERFACE_HPP_
