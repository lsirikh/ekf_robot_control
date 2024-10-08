#ifndef INCLUDE_EXTENDEDKALMANFILTER_HPP_
#define INCLUDE_EXTENDEDKALMANFILTER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include "measurement.hpp"
#include "rclcpp/rclcpp.hpp"

class ExtendedKalmanFilter
{
    public:
        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter();

        void predict(double dt, double twoDimensionalMode);
        void correct(Measurement &measurement);
        void setInitialErrorEstimateCovariance();
        void setProcessNoiseCovariance();
        void resetAngleOverflow();
        double clamp(double rotation);
        Eigen::VectorXd getStates();
        Eigen::MatrixXd getEstimateErrorCovariance();

    private:
        Eigen::VectorXd state_;
        Eigen::MatrixXd processNoiseCovariance_;
        Eigen::MatrixXd estimateErrorCovariance_;
        Eigen::MatrixXd transferFunction_;
        Eigen::MatrixXd transferFunctionJacobian_;
        Eigen::MatrixXd identity_;

        int stateSize = 15;
        int StateMemberX = 0;
        int StateMemberY = 1;
        int StateMemberZ = 2;
        int StateMemberRoll = 3;
        int StateMemberPitch = 4;
        int StateMemberYaw = 5;
        int StateMemberVx = 6;
        int StateMemberVy = 7;
        int StateMemberVz = 8;
        int StateMemberVroll = 9;
        int StateMemberVpitch = 10;
        int StateMemberVyaw = 11;
        int StateMemberAx = 12;
        int StateMemberAy = 13;
        int StateMemberAz = 14;
};

#endif  // INCLUDE_EXTENDEDKALMANFILTER_HPP_
