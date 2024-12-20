#include "ec_calculator/low_pass_filter.h"

namespace ec_calculator
{
    void LowPassFilter::setLowPassFilter(const int &value_size_, const int &save_size_)
    {
        _value_size = value_size_;
        _save_size = save_size_;
        _values.resize(_save_size);
        for (int i=0; i<_save_size; i++)
        {
            _values[i].resize(_value_size, 1);
            _values[i].setZero();
        }
        _average_value.resize(_value_size, 1);
    }

    void LowPassFilter::saveValue(const Eigen::Matrix<double, -1, 1> &value_)
    {
        rotate(_values.rbegin(), _values.rbegin()+1, _values.rend());
        _values[0] = value_;
    }

    Eigen::Matrix<double, -1, 1> LowPassFilter::averageMovingMethod(const Eigen::Matrix<double, -1, 1> &value_)
    {
        saveValue(value_);

        _average_value.setZero();
        for(int i=0; i<_save_size; i++)
        {
            _average_value += _values[i];
        }
        _average_value /= double(_save_size);

        return _average_value;
    }
}