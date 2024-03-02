#ifndef DIFFERENTIAL_INTEGRAL_H
#define DIFFERENTIAL_INTEGRAL_H

#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ec_calculator
{
    class DifferentialIntegral
    {
        private:
            bool _d_is_first = true;
            std::chrono::system_clock::time_point _d_start, _d_end;
            double _d_during;
            Eigen::Matrix<double, -1, 1> _d_value_old;
            Eigen::Matrix<double, -1, 1> _d_value;

            bool _i_is_first = true;
            std::chrono::system_clock::time_point _i_start, _i_end;
            double _i_during;
            Eigen::Matrix<double, -1, 1> _i_value_old;
            Eigen::Matrix<double, -1, 1> _i_value;

        public:
            Eigen::Matrix<double, -1, 1> getDValue();
            Eigen::Matrix<double, -1, 1> differential(const Eigen::Matrix<double, -1, 1> &now_value_);
            Eigen::Matrix<double, -1, 1> differentialN(const Eigen::Matrix<double, -1, 1> &now_value_, const int &differential_number_);

            Eigen::Matrix<double, -1, 1> getIValue();
            Eigen::Matrix<double, -1, 1> integral(const Eigen::Matrix<double, -1, 1> &now_value_);
    };
}

#endif