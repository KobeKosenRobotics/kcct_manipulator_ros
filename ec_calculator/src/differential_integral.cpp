#include "ec_calculator/differential_integral.h"

namespace ec_calculator
{
    Eigen::Matrix<double, -1, 1> DifferentialIntegral::getDValue()
    {
        return _d_value;
    }

    Eigen::Matrix<double, -1, 1> DifferentialIntegral::differential(const Eigen::Matrix<double, -1, 1> &now_value_)
    {
        if(_is_first)
        {
            _start = std::chrono::system_clock::now();
            _old_value = now_value_;
            _is_first = false;

            return 0.0*now_value_; // now_value_.setZero()
        }

        _end = std::chrono::system_clock::now();
        _during = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_end - _start).count();
        _start = _end;

        _d_value = (now_value_ - _old_value)/_during;
        _old_value = now_value_;

        return _d_value;
    }

    Eigen::Matrix<double, -1, 1> DifferentialIntegral::differentialN(const Eigen::Matrix<double, -1, 1> &now_value_, const int &differential_number_)
    {
        if(_is_first)
        {
            _start = std::chrono::system_clock::now();
            _old_value = now_value_;
            _is_first = false;

            return 0.0*now_value_;  // now_value_.setZero()
        }

        _end = std::chrono::system_clock::now();
        _during = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_end - _start).count();
        _start = _end;

        _d_value = (now_value_ - _old_value)/(pow(_during, differential_number_));
        _old_value = now_value_;

        return _d_value;
    }
}
