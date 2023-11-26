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
            bool _during_time_enable = false;
            double _linear_velocity = 0.01;  // [m/s]
            double _sigmoid_gain = 2;

            Eigen::Matrix<double, -1, 1> _sin_gain;

        public:
            void setLinearVelocity(const double &linear_velocity_);
            void setSigmoidGain(const double &sigmoid_gain_);
            void setDuringTime(const double &during_time_);
            void setPoint(const Eigen::Matrix<double, -1, 1> &start_point_, const Eigen::Matrix<double, -1, 1> &end_point_);
            double getDistance();
            double getDuringTime();
            double getMidTime();
            Eigen::Matrix<double, -1, 1> getLinearInterpolation();
            Eigen::Matrix<double, -1, 1> getSinInterpolation();
            Eigen::Matrix<double, -1, 1> getDSinInterpolation();
            Eigen::Matrix<double, -1, 1> getDDSinInterpolation();
            Eigen::Matrix<double, -1, 1> getSigmoidInterpolation();
    };
}

#define EC_CALCULATOR_INTERPOLATION_H
#endif