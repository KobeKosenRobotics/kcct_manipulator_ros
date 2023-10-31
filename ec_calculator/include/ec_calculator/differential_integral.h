#ifndef DIFFERENTIAL_INTEGRAL_H
#define DIFFERENTIAL_INTEGRAL_H

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
            bool _is_first = true;
            std::chrono::system_clock::time_point _start, _end;
            double _during;
            Eigen::Matrix<double, -1, 1> _old_value;
            Eigen::Matrix<double, -1, 1> _d_value;

        public:
            Eigen::Matrix<double, -1, 1> getDValue();
            Eigen::Matrix<double, -1, 1> differential(const Eigen::Matrix<double, -1, 1> &now_value_);
            Eigen::Matrix<double, -1, 1> differentialN(const Eigen::Matrix<double, -1, 1> &now_value_, const int &differential_number_);
    };
}

#endif