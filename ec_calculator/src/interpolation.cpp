#include "ec_calculator/interpolation.h"

namespace ec_calculator
{
    void Interpolation::setLinearVelocity(const double &linear_velocity_)
    {
        _linear_velocity = linear_velocity_;
    }

    void Interpolation::setSigmoidGain(const double &sigmoid_gain_)
    {
        _sigmoid_gain = sigmoid_gain_;
    }

    void Interpolation::setDuringTime(const double &during_time_)
    {
        _during_time_enable = true;

        _during_time = during_time_;
    }

    void Interpolation::setPoint(const Eigen::Matrix<double, -1, 1> &start_point_, const Eigen::Matrix<double, -1, 1> &end_point_)
    {
        _start_point = start_point_;
        _end_point = end_point_;

        _distance = getDistance();

        _during_time = getDuringTime();

        if(0.0 < _during_time) _sin_gain = ((2.0*M_PI)/(_during_time*_during_time))*(_end_point - _start_point);
        else _sin_gain = 0.0*_start_point;

        _start_time = std::chrono::system_clock::now();
    }

    double Interpolation::getDistance()
    {
        _distance = (_end_point - _start_point).norm();

        return _distance;
    }

    double Interpolation::getDuringTime()
    {
        if(_during_time_enable) return _during_time;

        _during_time = _distance/_linear_velocity;

        return _during_time;
    }

    double Interpolation::getMidTime()
    {
        _mid_time = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-_start_time).count()/_during_time;
        _mid_time = std::min(std::max((_mid_time), 0.0), 1.0);
        if(std::isnan(_mid_time)) _mid_time = 0.0;

        return _mid_time;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getStartPoint()
    {
        return _start_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getEndPoint()
    {
        return _end_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getLinearInterpolation()
    {
        getMidTime();
        _mid_point = _start_point * (1 - _mid_time) + _end_point * _mid_time;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getDLinearInterpolation()
    {
        getMidTime();
        if(_mid_time >= 1.0)
        {
            return 0.0*_mid_point;
        }

        return (_end_point - _start_point)/_during_time;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getSinInterpolation()
    {
        getMidTime();
        _mid_time *= (2.0*M_PI);

        _mid_point = pow((_during_time/(2.0*M_PI)), 2)*(-sin(_mid_time) + _mid_time)*_sin_gain + _start_point;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getDSinInterpolation()
    {
        getMidTime();
        _mid_time *= (2.0*M_PI);

        _mid_point = (_during_time/(2.0*M_PI))*(-cos(_mid_time) + 1.0)*_sin_gain;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getDDSinInterpolation()
    {
        getMidTime();
        _mid_time *= (2.0*M_PI);

        _mid_point = (sin(_mid_time))*_sin_gain;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getSigmoidInterpolation()
    {
        getMidTime();
        double A_ = exp(-_sigmoid_gain*(2*_mid_time-1));
        double B_ = exp(-_sigmoid_gain);
        _mid_time = 0.5*(1 + (1-A_)/(1+A_)*(1+B_)/(1-B_));
        _mid_point = _start_point * (1 - _mid_time) + _end_point * _mid_time;

        return _mid_point;
    }
}