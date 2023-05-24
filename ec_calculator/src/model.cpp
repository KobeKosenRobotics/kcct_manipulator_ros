#include "ec_calculator/model.h"

namespace ec_calculator
{
    // Constructor
    Model::Model()
    {
        _chain_num = 3;
        _joint_num = 10;

        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
        // 1  2  3  4  5  6  7  8  9 10
        1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
        1, 1, 0, 0, 1, 0, 0, 1, 0, 0;

        _joint_position_link.resize(3, (_joint_num + _chain_num));
        _joint_position_link <<
        1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
        0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _translation_axis.resize(3, _joint_num);
        _translation_axis <<
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 0, 1, 1, 1;

        _angle_2_angular_velocity_gain.resize(_joint_num, _joint_num);
        _angle_2_angular_velocity_gain.setIdentity(_joint_num, _joint_num);

        _ec_gain = 1.0;
    }

    // Change Model
    void Model::changeModel(const int &chain_num_,
                            const int &joint_num_,
                            const Eigen::Matrix<bool, -1, -1> &chain_mat_,
                            const Eigen::Matrix<double, 3, -1> &joint_position_link_,
                            const Eigen::Matrix<double, 3, -1> &translation_axis_,
                            const Eigen::Matrix<double, 3, -1> &rotation_axis_,
                            const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_,
                            const double &ec_gain_)
    {
        // TODO: If any of the parameters cannot be changed, emergency stop change true.
        changeChainNum(chain_num_);
        changeJointNum(joint_num_);
        changeChainMatrix(chain_mat_);
        changeJointPositionLink(joint_position_link_);
        changeTranslationAxis(translation_axis_);
        changeRotationAxis(rotation_axis_);
        changeAngle2AngularVelocityGain(angle_2_angular_velocity_gain_);
        changeECGain(ec_gain_);
    }

    void Model::changeChainNum(const int &chain_num_)
    {
        _chain_num = chain_num_;
    }

    void Model::changeJointNum(const int &joint_num_)
    {
        _joint_num = joint_num_;
    }

    void Model::changeChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_mat_)
    {
        if(chain_mat_.rows() == _chain_num && chain_mat_.cols() == _joint_num)
        {
            _chain_mat.resize(_chain_num, _joint_num);
            _chain_mat = chain_mat_;
        }
        else
        {
            std::cout << "Matrix(chain_mat_) Size do not match Chain Number or Joint Number." << std::endl;
        }
    }

    void Model::changeJointPositionLink(const Eigen::Matrix<double, 3, -1> &joint_position_link_)
    {
        if(joint_position_link_.cols() == (_joint_num + _chain_num))
        {
            _joint_position_link.resize(3, (_joint_num + _chain_num));
            _joint_position_link = joint_position_link_;
        }
        else
        {
            std::cout << "Matrix(joint_position_link_) Size do not match Chain Number or Joint Number." << std::endl;
        }
    }

    void Model::changeCenterOfGravityLink(const Eigen::Matrix<double, 3, -1> &center_of_gravity_link_)
    {
        if(center_of_gravity_link_.cols() == _joint_num)
        {
            _center_of_gravity_link.resize(3, _joint_num);
            _center_of_gravity_link = center_of_gravity_link_;
        }
        else
        {
            std::cout << "Matrix(center_of_gravity_link_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeTranslationAxis(const Eigen::Matrix<double, 3, -1> &translation_axis_)
    {
        if(translation_axis_.cols() == _joint_num)
        {
            _translation_axis.resize(3, _joint_num);
            _translation_axis = translation_axis_;
        }
        else
        {
            std::cout << "Matrix(translation_axis_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeRotationAxis(const Eigen::Matrix<double, 3, -1> &rotation_axis_)
    {
        if(rotation_axis_.cols() == _joint_num)
        {
            _rotation_axis.resize(3, _joint_num);
            _rotation_axis = rotation_axis_;
        }
        else
        {
            std::cout << "Matrix(rotation_axis_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeAngle2AngularVelocityGain(const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_)
    {
        if(angle_2_angular_velocity_gain_.rows() == _joint_num && angle_2_angular_velocity_gain_.cols() == _joint_num)
        {
            _angle_2_angular_velocity_gain.resize(_joint_num, _joint_num);
            _angle_2_angular_velocity_gain = angle_2_angular_velocity_gain_;
        }
        else
        {
            std::cout << "Matrix(angle_2_angular_velocity_gain_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeECGain(const double &ec_gain_)
    {
        _ec_gain = ec_gain_;
    }

    // Parameter Getters
    int Model::getJointNum()
    {
        return _joint_num;
    }

    int Model::getChainNum()
    {
        return _chain_num;
    }

    Eigen::Matrix<bool, -1, -1> Model::getChainMat()
    {
        return _chain_mat;
    }

    Eigen::Matrix<double, 3, 1> Model::getJointPositionLink(const int &joint_)
    {
        if(joint_ < 0 || (_joint_num + _chain_num) <= joint_) return Eigen::Matrix<double, 3, 1>::Zero();
        return _joint_position_link.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getCenterOfGravityLink(const int &joint_)
    {
        if(joint_ < 0 || _joint_num <= joint_) return Eigen::Matrix<double, 3, 1>::Zero();
        return _center_of_gravity_link.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getTranslationAxis(const int &joint_)
    {
        if(joint_ < 0 || _joint_num <= joint_) return Eigen::Matrix<double, 3, 1>::Zero();
        return _translation_axis.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getRotationAxis(const int &joint_)
    {
        if(joint_ < 0 || _joint_num <= joint_) return Eigen::Matrix<double, 3, 1>::Zero();
        return _rotation_axis.col(joint_);
    }

    Eigen::Matrix<double, -1, -1> Model::getAngle2AngularVelocityGain()
    {
        return _angle_2_angular_velocity_gain;
    }

    double Model::getECGain()
    {
        return _ec_gain;
    }
}