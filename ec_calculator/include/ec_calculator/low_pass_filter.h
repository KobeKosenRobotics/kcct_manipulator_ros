#ifndef EC_CALCULATOR_LOW_PASS_FILTER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>

namespace ec_calculator
{
    class LowPassFilter
    {
        public:
            void setLowPassFilter(const int &value_size_, const int &save_size_);
            void saveValue(const Eigen::Matrix<double, -1, 1> &value_);
            Eigen::Matrix<double, -1, 1> averageMovingMethod(const Eigen::Matrix<double, -1, 1> &value_);
        private:
            int _value_size;
            int _save_size;
            std::vector<Eigen::Matrix<double, -1, 1>> _values;
            Eigen::Matrix<double, -1, 1> _average_value;
    };
}

#define EC_CALCULATOR_LOW_PASS_FILTER_H
#endif