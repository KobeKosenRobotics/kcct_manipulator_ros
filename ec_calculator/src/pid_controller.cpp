#include "ec_calculator/pid_controller.h"

namespace ec_calculator
{
    PidController::PidController()
    {
        _scalar_gains.resize(3);
        _matrix_gains.resize(3);
    }
    void PidController::setScalarGains(const std::vector<double> &scalar_gains_)
    {
        _scalar_gains = scalar_gains_;

        _matrix_gains[0].resize(1,1);
        _matrix_gains[0].setZero();
        _matrix_gains[1].resize(1,1);
        _matrix_gains[1].setZero();
        _matrix_gains[2].resize(1,1);
        _matrix_gains[2].setZero();
    }

    void PidController::setMatrixGains(const std::vector<Eigen::Matrix<double, -1, -1>> &matrix_gains_)
    {
        _matrix_gains = matrix_gains_;

        _scalar_gains[0] = 0.0;
        _scalar_gains[1] = 0.0;
        _scalar_gains[2] = 0.0;
    }

    void PidController::setPGain(const double &p_gain_)
    {
        _scalar_gains[0] = p_gain_;
    }

    void PidController::setIGain(const double &i_gain_)
    {
        _scalar_gains[1] = i_gain_;
    }

    void PidController::setDGain(const double &d_gain_)
    {
        _scalar_gains[2] = d_gain_;
    }

    void PidController::setPGain(const Eigen::Matrix<double, -1, -1> &p_gain_)
    {
        _matrix_gains[0] = p_gain_;
    }

    void PidController::setIGain(const Eigen::Matrix<double, -1, -1> &i_gain_)
    {
        _matrix_gains[1] = i_gain_;
    }

    void PidController::setDGain(const Eigen::Matrix<double, -1, -1> &d_gain_)
    {
        _matrix_gains[2] = d_gain_;
    }

    double PidController::getPid(const double &error_)
    {
        std::cout << "This function has not yet been implemented." << std::endl;

        return 0.0;
    }

    Eigen::Matrix<double, -1, 1> PidController::getPid(const Eigen::Matrix<double, -1, 1> &error_)
    {
        _p_error = error_;
        _i_error = _integral.integral(error_);
        _d_error = _differential.differential(error_);

        if(error_.rows() == _matrix_gains[0].rows())
        {
            return _matrix_gains[0]*_p_error + _matrix_gains[1]*_i_error + _matrix_gains[2]*_d_error;
        }

        return _scalar_gains[0]*_p_error + _scalar_gains[1]*_i_error + _scalar_gains[2]*_d_error;
    }

    Eigen::Matrix<double, -1, 1> PidController::getError()
    {
        return _p_error;
    }
}
