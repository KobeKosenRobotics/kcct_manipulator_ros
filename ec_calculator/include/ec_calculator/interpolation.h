#ifndef EC_CALCULATOR_INTERPOLATION_H

#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ec_calculator
{
    class Interpolation
    {
        private:
            Eigen::Matrix<double, -1, 1> _start_point;
            Eigen::Matrix<double, -1, 1> _end_point;
            Eigen::Matrix<double, -1, 1> _mid_point;

            std::chrono::system_clock::time_point _start_time;
            double _mid_time;

            double _distance;
            double _during_time;
            double _linear_velocity = 0.1;  // [m/s]
            double _sigmoid_gain = 2;

        public:
            void setLinearVelocity(const double &linear_velocity_);
            void setSigmoidGain(const double &sigmoid_gain_);
            void setPoint(const Eigen::Matrix<double, -1, 1> &start_point_, const Eigen::Matrix<double, -1, 1> &end_point_);
            double getDistance();
            double getDuringTime();
            double getMidTime();
            Eigen::Matrix<double, -1, 1> getLinearInterpolation();
            Eigen::Matrix<double, -1, 1> getCosInterpolation();
            Eigen::Matrix<double, -1, 1> getDCosInterpolation();
            Eigen::Matrix<double, -1, 1> getDDCosInterpolation();
            Eigen::Matrix<double, -1, 1> getSigmoidInterpolation();
            Eigen::Matrix<double, -1, 1> getDSigmoidInterpolation();
            Eigen::Matrix<double, -1, 1> getDDSigmoidInterpolation();
    };
}

#define EC_CALCULATOR_INTERPOLATION_H
#endif