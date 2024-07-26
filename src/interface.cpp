#include "ekf_robot_control/interface.hpp"

Interface::Interface(bool imu, bool odometry) : Node("interface_node"), useIMU_(imu), useOdometry_(odometry) {
    imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 1, std::bind(&Interface::callbackIMU, this, std::placeholders::_1));
    odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Interface::callbackOdometry, this, std::placeholders::_1));
    odometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/fused", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    fuseData();
}

Interface::~Interface() {
}

void Interface::callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg){
    Eigen::VectorXd measurement(stateSize_);
    Eigen::VectorXd measurementCovariance(stateSize_);
    measurement.setZero();
    measurementCovariance.setZero();
    data_.getAccelerationMeasurement(*msg, ekf_, measurement, measurementCovariance, twoDimensionalMode);
    
    // Log IMU measurements and covariances
    // RCLCPP_INFO(this->get_logger(), "IMU Measurements: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, vx: %f, vy: %f, vz: %f, vroll: %f, vpitch: %f, vyaw: %f, ax: %f, ay: %f, az: %f]",
    //             measurement[0], measurement[1], measurement[2], measurement[3], measurement[4], measurement[5],
    //             measurement[6], measurement[7], measurement[8], measurement[9], measurement[10], measurement[11],
    //             measurement[12], measurement[13], measurement[14]);
    // RCLCPP_INFO(this->get_logger(), "IMU Measurement Covariances: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, vx: %f, vy: %f, vz: %f, vroll: %f, vpitch: %f, vyaw: %f, ax: %f, ay: %f, az: %f]",
    //             measurementCovariance[0], measurementCovariance[1], measurementCovariance[2], measurementCovariance[3], measurementCovariance[4], measurementCovariance[5],
    //             measurementCovariance[6], measurementCovariance[7], measurementCovariance[8], measurementCovariance[9], measurementCovariance[10], measurementCovariance[11],
    //             measurementCovariance[12], measurementCovariance[13], measurementCovariance[14]);

    storeMeasurement(measurement, measurementCovariance, dataIMU);
    firstMeasurement = true;
}

void Interface::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg){
    Eigen::VectorXd measurement(stateSize_);
    Eigen::VectorXd measurementCovariance(stateSize_);
    measurement.setZero();
    measurementCovariance.setZero();
    data_.getOdometryMeasurement(*msg, ekf_, measurement, measurementCovariance, twoDimensionalMode);

    // Log Odometry measurements and covariances
    // RCLCPP_INFO(this->get_logger(), "Odometry Measurements: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, vx: %f, vy: %f, vz: %f, vroll: %f, vpitch: %f, vyaw: %f, ax: %f, ay: %f, az: %f]",
    //             measurement[0], measurement[1], measurement[2], measurement[3], measurement[4], measurement[5],
    //             measurement[6], measurement[7], measurement[8], measurement[9], measurement[10], measurement[11],
    //             measurement[12], measurement[13], measurement[14]);
    // RCLCPP_INFO(this->get_logger(), "Odometry Measurement Covariances: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, vx: %f, vy: %f, vz: %f, vroll: %f, vpitch: %f, vyaw: %f, ax: %f, ay: %f, az: %f]",
    //             measurementCovariance[0], measurementCovariance[1], measurementCovariance[2], measurementCovariance[3], measurementCovariance[4], measurementCovariance[5],
    //             measurementCovariance[6], measurementCovariance[7], measurementCovariance[8], measurementCovariance[9], measurementCovariance[10], measurementCovariance[11],
    //             measurementCovariance[12], measurementCovariance[13], measurementCovariance[14]);

    storeMeasurement(measurement, measurementCovariance, dataOdometry);
    firstMeasurement = true;
}

void Interface::storeMeasurement(Eigen::VectorXd &measurements, Eigen::VectorXd &measurementCovariances, std::vector<int> data){
    // RCLCPP_INFO(this->get_logger(), "Storing Measurement:");
    // RCLCPP_INFO(this->get_logger(), "Measurements: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, vx: %f, vy: %f, vz: %f, vroll: %f, vpitch: %f, vyaw: %f, ax: %f, ay: %f, az: %f]",
    //             measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5],
    //             measurements[6], measurements[7], measurements[8], measurements[9], measurements[10], measurements[11],
    //             measurements[12], measurements[13], measurements[14]);
    // RCLCPP_INFO(this->get_logger(), "Measurement Covariances: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, vx: %f, vy: %f, vz: %f, vroll: %f, vpitch: %f, vyaw: %f, ax: %f, ay: %f, az: %f]",
    //             measurementCovariances[0], measurementCovariances[1], measurementCovariances[2], measurementCovariances[3], measurementCovariances[4], measurementCovariances[5],
    //             measurementCovariances[6], measurementCovariances[7], measurementCovariances[8], measurementCovariances[9], measurementCovariances[10], measurementCovariances[11],
    //             measurementCovariances[12], measurementCovariances[13], measurementCovariances[14]);
    // RCLCPP_INFO(this->get_logger(), "Data to Use: [x: %d, y: %d, z: %d, roll: %d, pitch: %d, yaw: %d, vx: %d, vy: %d, vz: %d, vroll: %d, vpitch: %d, vyaw: %d, ax: %d, ay: %d, az: %d]",
    //             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14]);
    
    sensor_.measurements = measurements;
    sensor_.measurementCovariances = measurementCovariances;
    sensor_.dataToUse = data;
    sensorMeasurements_.push_back(sensor_);
    ekf_.correct(sensor_);
}

void Interface::fuseData(){
    double frequency = 10;
    rclcpp::Rate loop(frequency);
    RCLCPP_INFO(this->get_logger(), "Fusing");
    while(rclcpp::ok()){
        if(firstMeasurement) {
            double dt = 1/frequency;
            ekf_.predict(dt, twoDimensionalMode);
            Eigen::VectorXd states = ekf_.getStates();
            publishOdometry(states);
            publishTransform(states);
            sensorMeasurements_.clear();
            loop.sleep();
        }
        rclcpp::spin_some(this->get_node_base_interface());
    }
}

void Interface::publishOdometry(Eigen::VectorXd states){
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.header.stamp = this->now();
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = states[0];
    odometry.pose.pose.position.y = states[1];
    odometry.pose.pose.position.z = states[2];

    odometry.twist.twist.linear.x = states[6];
    odometry.twist.twist.linear.y = states[7];
    odometry.twist.twist.linear.z = states[8];
    odometry.twist.twist.angular.x = states[9];
    odometry.twist.twist.angular.y = states[10];
    odometry.twist.twist.angular.z = states[11];

    tf2::Quaternion q;
    q.setRPY(states[3], states[4], states[5]);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();

    Eigen::MatrixXd estimateErrorCovariance_ = ekf_.getEstimateErrorCovariance();
    odometry.pose.covariance[0] = estimateErrorCovariance_(0, 0);
    odometry.pose.covariance[7] = estimateErrorCovariance_(1, 1);
    odometry.pose.covariance[14] = estimateErrorCovariance_(2, 2);   
    odometry.pose.covariance[21] = estimateErrorCovariance_(3, 3);
    odometry.pose.covariance[28] = estimateErrorCovariance_(4, 4);
    odometry.pose.covariance[35] = estimateErrorCovariance_(5, 5);
    odometry.twist.covariance[0] = estimateErrorCovariance_(6, 6);
    odometry.twist.covariance[7] = estimateErrorCovariance_(7, 7);
    odometry.twist.covariance[14] = estimateErrorCovariance_(8, 8);   
    odometry.twist.covariance[21] = estimateErrorCovariance_(9, 9);
    odometry.twist.covariance[28] = estimateErrorCovariance_(10, 10);
    odometry.twist.covariance[35] = estimateErrorCovariance_(11, 11); 
    odometryPub_->publish(odometry);
}

void Interface::publishTransform(Eigen::VectorXd state){
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = state(0);
    transformStamped.transform.translation.y = state(1);
    transformStamped.transform.translation.z = state(2);
    tf2::Quaternion q;
    q.setRPY(state(3), state(4), state(5));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transformStamped);

    // base_link to base_footprint transform
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q_base_footprint(0, 0, 0, 1);
    transformStamped.transform.rotation.x = q_base_footprint.x();
    transformStamped.transform.rotation.y = q_base_footprint.y();
    transformStamped.transform.rotation.z = q_base_footprint.z();
    transformStamped.transform.rotation.w = q_base_footprint.w();
    tf_broadcaster_->sendTransform(transformStamped);

    // base_link to laser transform
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "laser";
    transformStamped.transform.translation.x = 0.0; // Adjust according to your robot's configuration
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0.1; // Adjust according to your robot's configuration
    tf2::Quaternion q_laser(0, 0, 0, 1);
    transformStamped.transform.rotation.x = q_laser.x();
    transformStamped.transform.rotation.y = q_laser.y();
    transformStamped.transform.rotation.z = q_laser.z();
    transformStamped.transform.rotation.w = q_laser.w();
    tf_broadcaster_->sendTransform(transformStamped);
}
