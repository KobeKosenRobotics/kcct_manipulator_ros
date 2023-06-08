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

    void Interpolation::setPoint(const Eigen::Matrix<double, -1, 1> &start_point_, const Eigen::Matrix<double, -1, 1> &end_point_)
    {
        _start_point = start_point_;
        _end_point = end_point_;

        _distance = getDistance();

        _during_time = getDuringTime();

        _start_time = std::chrono::system_clock::now();
    }

    double Interpolation::getDistance()
    {
        _distance = (_end_point - _start_point).norm();

        return _distance;
    }

    double Interpolation::getDuringTime()
    {
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

    Eigen::Matrix<double, -1, 1> Interpolation::getLinearInterpolation()
    {
        getMidTime();
        _mid_point = _start_point * (1 - _mid_time) + _end_point * _mid_time;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getCosInterpolation()
    {
        getMidTime();
        _mid_time = (1 - cos(_mid_time * M_PI))/2.0;
        _mid_point = _start_point * (1 - _mid_time) + _end_point * _mid_time;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getDCosInterpolation()
    {
        getMidTime();
        _mid_point = (M_PI*sin(M_PI*_mid_time)*(_end_point - _start_point))/2.0;

        return _mid_point;
    }

    Eigen::Matrix<double, -1, 1> Interpolation::getDDCosInterpolation()
    {
        getMidTime();
        _mid_point = (M_PI*M_PI*cos(M_PI*_mid_time)*(_end_point - _start_point))/2.0;

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

    Eigen::Matrix<double, -1, 1> Interpolation::getDSigmoidInterpolation()
    {
        getMidTime();
        _mid_point = -(2*_sigmoid_gain*exp(_sigmoid_gain*(2*_mid_time - 1))*(exp(_sigmoid_gain) + 1)*(_end_point - _start_point))/(2*exp(_sigmoid_gain*(2*_mid_time - 1)) - exp(_sigmoid_gain*(4*_mid_time - 1)) + exp(2*_sigmoid_gain*(2*_mid_time - 1)) - 2*exp(2*_mid_time*_sigmoid_gain) - exp(_sigmoid_gain) + 1);

        return _mid_point;
    }

    // Eigen::Matrix<double, -1, 1> Interpolation::getSigmoidInterpolation()
    // {
        // getMidTime();
        // double A_ = exp(-_sigmoid_gain*(2*_mid_time-1));
        // double B_ = exp(-_sigmoid_gain);
        // _mid_time = 0.5*(1 + (1-A_)/(1+A_)*(1+B_)/(1-B_));
        // _mid_point = _start_point * (1 - _mid_time) + _end_point * _mid_time;

        // return _mid_point;
    // }
}