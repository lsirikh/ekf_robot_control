#ifndef INCLUDE_MEASUREMENT_HPP_
#define INCLUDE_MEASUREMENT_HPP_

#include <Eigen/Dense>
#include <vector>

struct Measurement
{
    Eigen::VectorXd measurements;
    Eigen::VectorXd measurementCovariances;
    std::vector<int> dataToUse;
};

#endif  //  INCLUDE_MEASUREMENT_HPP_
