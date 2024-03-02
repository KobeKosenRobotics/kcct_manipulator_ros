#ifndef EC_CALCULATOR_PID_CONTROLLER_H

#include "differential_integral.h"

#include <iostream>
#include <chrono>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ec_calculator
{
    class PidController
    {
        private:
            Eigen::Matrix<double, -1, 1> _p_error;
            Eigen::Matrix<double, -1, 1> _i_error;
            Eigen::Matrix<double, -1, 1> _d_error;

            DifferentialIntegral _integral;
            DifferentialIntegral _differential;
            DifferentialIntegral _2differential;

            std::vector<double> _scalar_gains;
            std::vector<Eigen::Matrix<double, -1, -1>> _matrix_gains;

        public:
            PidController();

            void setScalarGains(const std::vector<double> &scalar_gains_);
                void setPGain(const double &p_gain_);
                void setIGain(const double &i_gain_);
                void setDGain(const double &d_gain_);
            void setMatrixGains(const std::vector<Eigen::Matrix<double, -1, -1>> &matrix_gains_);
                void setPGain(const Eigen::Matrix<double, -1, -1> &p_gain_);
                void setIGain(const Eigen::Matrix<double, -1, -1> &i_gain_);
                void setDGain(const Eigen::Matrix<double, -1, -1> &d_gain_);
            double getPid(const double &error_);
            Eigen::Matrix<double, -1, 1> getPid(const Eigen::Matrix<double, -1, 1> &error_);
            Eigen::Matrix<double, -1, 1> getError();
    };
}

#define EC_CALCULATOR_PID_CONTROLLER_H
#endif