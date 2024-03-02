#include "ec_calculator/differential_integral.h"

namespace ec_calculator
{
    Eigen::Matrix<double, -1, 1> DifferentialIntegral::getDValue()
    {
        return _d_value;
    }

    Eigen::Matrix<double, -1, 1> DifferentialIntegral::differential(const Eigen::Matrix<double, -1, 1> &now_value_)
    {
        if(_d_is_first)
        {
            _d_start = std::chrono::system_clock::now();
            _d_value_old = now_value_;
            _d_value = 0.0*now_value_;
            _d_is_first = false;

            return _d_value;
        }

        _d_end = std::chrono::system_clock::now();
        _d_during = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_d_end - _d_start).count();
        _d_start = _d_end;

        _d_value = (now_value_ - _d_value_old)/_d_during;
        _d_value_old = now_value_;

        return _d_value;
    }

    Eigen::Matrix<double, -1, 1> DifferentialIntegral::differentialN(const Eigen::Matrix<double, -1, 1> &now_value_, const int &differential_number_)
    {
        if(_d_is_first)
        {
            _d_start = std::chrono::system_clock::now();
            _d_value_old = now_value_;
            _d_value = 0.0*now_value_;
            _d_is_first = false;

            return _d_value;
        }

        _d_end = std::chrono::system_clock::now();
        _d_during = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_d_end - _d_start).count();
        _d_start = _d_end;

        _d_value = (now_value_ - _d_value_old)/(pow(_d_during, differential_number_));
        _d_value_old = now_value_;

        return _d_value;
    }

    Eigen::Matrix<double, -1, 1> DifferentialIntegral::getIValue()
    {
        return _i_value;
    }

    Eigen::Matrix<double, -1, 1> DifferentialIntegral::integral(const Eigen::Matrix<double, -1, 1> &now_value_)
    {
        if(_i_is_first)
        {
            _i_start = std::chrono::system_clock::now();
            _i_value_old = now_value_;
            _i_value = 0.0*now_value_;
            _i_is_first = false;

            return _i_value;
        }

        _i_end = std::chrono::system_clock::now();
        _i_during = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_i_end - _i_start).count();
        _i_start = _i_end;

        _i_value = (now_value_ - _i_value_old)*_i_during;
        _i_value_old = now_value_;

        return _i_value;
    }
}
