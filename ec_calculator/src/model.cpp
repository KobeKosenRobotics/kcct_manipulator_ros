#include "ec_calculator/model.h"

namespace ec_calculator
{
    // Constructor
    Model::Model()
    {
        /* open manipulator *//*
        _torque_control_enable = true;

        _chain_num = 1;
        _joint_num = 6;

        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
            1, 1, 1, 1, 1, 1;

        _joint_position_link.resize(3, (_joint_num + _chain_num));
        _joint_position_link <<
              0.0, 0.0,  30.0, - 30.0, 0.0,   0.0, 0.0,
              0.0, 0.0,   0.0,    0.0, 0.0,   0.0, 0.0,
            159.0, 0.0, 264.0,  258.0, 0.0, 123.0, 0.0;
        _joint_position_link *= 0.001;

        _translation_axis.resize(3, _joint_num);
        _translation_axis <<
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
            0, 0, 0, 0, 0, 0,
            0, 1, 1, 0, 1, 0,
            1, 0, 0, 1, 0, 1;

        if(_torque_control_enable)
        {
            _inertia.resize(10, _joint_num);
            _inertia.setConstant(1.0);
            _inertia.transpose() <<
                1030,
                1.4957303e+06,  0.0000000e+00,  0.0000000e+00,
                0.0000000e+00,  4.5009641e+05, -1.0959043e+04,
                0.0000000e+00, -1.0959043e+04,  1.4874997e+06,
                1404,
                 1.0627201e+07, 1.2357497e+04, -1.2920605e+06,
                 1.2357497e+04, 1.0014640e+07,  1.5798255e+05,
                -1.2920605e+06, 1.5798255e+05,  1.9568681e+06,
                1236,
                 3.1318491e+06, -6.0760429e+03, 2.4765806e+04,
                -6.0760429e+03,  2.9193915e+06, 4.2823763e+04,
                 2.4765806e+04,  4.2823763e+04, 9.2402606e+05,
                491,
                 3.9670485e+05, -3.3867048e+00, -4.7608394e+01,
                -3.3867048e+00,  2.3556702e+05,  3.9098238e+03,
                -4.7608394e+01,  3.9098238e+03,  2.9647894e+05,
                454,
                4.7548066e+05, 0.0000000e+00, 0.0000000e+00,
                0.0000000e+00, 3.9961989e+05, 1.4840847e+04,
                0.0000000e+00, 1.4840847e+04, 1.9795791e+05,
                5.0,
                5.0, 0.0, 0.0,
                0.0, 5.0, 0.0,
                0.0, 0.0, 5.0;
            _inertia.block(0, 0, 1, _joint_num) *= 0.001;
            _inertia.block(1, 0, 9, _joint_num) *= std::pow(0.001, 3);

            _center_of_gravity_link.resize(3, _joint_num);
            _center_of_gravity_link.transpose() <<
                  0  ,  1  , -  1.1,    // gl0 = g00
                 17.9,  0.3,  206.9,    // gl1 = g01
                -29.8,  0.3,  123.9,    // gl2 = g02 + q0 - q2 = g02 - l02 = g02[0.2,0.3,387.9] - [30,0,264]
                  0  , -1.5, -  7.7,    // gl3 = g03 + q0 - q3 = g03 - l03 = g03 - [0,0,264+258] = g03[0,-1.5,514.3] - [0,0,522]
                  0  ,  0.8,   69.5,    // gl4 = g04 + q0 - q4 = g04 - l04 = g04[0,0.8,591.5] - [0,0,522]
                  0  ,  0  ,    0  ;    // gl5 = g05 = 0
            _center_of_gravity_link *= 0.001;

            _angle_torque_control_p_gain = 0.2;
            _angle_torque_control_d_gain = 0.2;

            _gravitational_acceleration = 9.8;
        }

        _angle_velocity_control_p_gain.resize(_joint_num, _joint_num);
        _angle_velocity_control_p_gain.setIdentity(_joint_num, _joint_num);
        _angle_velocity_control_p_gain *= 0.1;

        _pose_velocity_control_p_gain = 0.5;
        */

        /* 1 Arm 3 fingers */
        _torque_control_enable = false;

        _chain_num = 3;
        _joint_num = 15;
        _binding_conditions = 3;

        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
            1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0,
            1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1;

        _joint_position_link.resize(3, (_joint_num + _chain_num));
        _joint_position_link <<
              0.0, 0.0,  30.0, - 30.0, 0.0,   0.0, 80.0*cos(0.0*M_PI/3.0),  0.0,  0.0, 80.0*cos(2.0*M_PI/3.0),  0.0,  0.0, 80.0*cos(4.0*M_PI/3.0),  0.0,  0.0,  0.0,  0.0,  0.0,
              0.0, 0.0,   0.0,    0.0, 0.0,   0.0, 80.0*sin(0.0*M_PI/3.0),  0.0,  0.0, 80.0*sin(2.0*M_PI/3.0),  0.0,  0.0, 80.0*sin(4.0*M_PI/3.0),  0.0,  0.0,  0.0,  0.0,  0.0,
            159.0, 0.0, 264.0,  258.0, 0.0, 123.0,                  160.0, 50.0, 50.0,                  160.0, 50.0, 50.0,                  160.0, 50.0, 50.0, 50.0, 50.0, 50.0;
        _joint_position_link *= 0.001;

        _translation_axis.resize(3, _joint_num);
        _translation_axis.setZero();

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
            0, 0, 0, 0, 0, 0,  sin(0.0*M_PI/3.0),  sin(0.0*M_PI/3.0),  sin(0.0*M_PI/3.0),  sin(2.0*M_PI/3.0),  sin(2.0*M_PI/3.0),  sin(2.0*M_PI/3.0),  sin(4.0*M_PI/3.0),  sin(4.0*M_PI/3.0),  sin(4.0*M_PI/3.0),
            0, 1, 1, 0, 1, 0, -cos(0.0*M_PI/3.0), -cos(0.0*M_PI/3.0), -cos(0.0*M_PI/3.0), -cos(2.0*M_PI/3.0), -cos(2.0*M_PI/3.0), -cos(2.0*M_PI/3.0), -cos(4.0*M_PI/3.0), -cos(4.0*M_PI/3.0), -cos(4.0*M_PI/3.0),
            1, 0, 0, 1, 0, 1,                  0,                  0,                  0,                  0,                  0,                  0,                  0,                  0,                  0;
            // cost -sint 0
            // sint  cost 0
            // 0     0    1

        _angle_velocity_control_p_gain.resize(_joint_num, _joint_num);
        _angle_velocity_control_p_gain.setIdentity(_joint_num, _joint_num);
        _angle_velocity_control_p_gain *= 0.1;

        _pose_velocity_control_p_gain = 0.5;

        _angle_limit.resize(2, _joint_num);
        _angle_limit <<
            -M_PI, -M_PI/2.0, -    M_PI/2.0, -M_PI, -M_PI/2.0, -M_PI, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0,
            +M_PI, +M_PI/2.0, +3.0*M_PI/4.0, +M_PI, +M_PI/2.0, +M_PI, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0, +M_PI/9.0;

        _angular_velocity_limit.resize(2, _joint_num);
        _angular_velocity_limit.setZero();

        _angular_acceleration_limit.resize(2, _joint_num);
        _angular_acceleration_limit.setZero();

        _jacobian_determinant_limit = 1.0e-30;

        /* 1 Arm 1 fingers *//*
        _torque_control_enable = false;

        _chain_num = 1;
        _joint_num = 9;
        _binding_conditions = 3;

        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
            1, 1, 1, 1, 1, 1, 1, 1, 1;

        _joint_position_link.resize(3, (_joint_num + _chain_num));
        _joint_position_link <<
              0.0, 0.0,  30.0, - 30.0, 0.0,   0.0, -100.0,  0.0,  0.0,  0.0,
              0.0, 0.0,   0.0,    0.0, 0.0,   0.0,    0.0,  0.0,  0.0,  0.0,
            159.0, 0.0, 264.0,  258.0, 0.0, 123.0,  200.0, 50.0, 50.0, 50.0;
        _joint_position_link *= 0.001;

        _translation_axis.resize(3, _joint_num);
        _translation_axis.setZero();

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
            0, 0, 0, 0, 0, 0, -sin(0.0*M_PI/3.0), -sin(0.0*M_PI/3.0), -sin(0.0*M_PI/3.0),
            0, 1, 1, 0, 1, 0,  cos(0.0*M_PI/3.0),  cos(0.0*M_PI/3.0),  cos(0.0*M_PI/3.0),
            1, 0, 0, 1, 0, 1,                  0,                  0,                  0;
            // cost -sint 0
            // sint  cost 0
            // 0     0    1

        _angle_velocity_control_p_gain.resize(_joint_num, _joint_num);
        _angle_velocity_control_p_gain.setIdentity(_joint_num, _joint_num);
        _angle_velocity_control_p_gain *= 0.1;

        _pose_velocity_control_p_gain = 0.5;

        _angle_limit.resize(2, _joint_num);
        _angle_limit <<
            -M_PI, -M_PI/2.0, -    M_PI/2.0, -M_PI, -M_PI/2.0, -M_PI, -M_PI/3.0, -M_PI/3.0, -M_PI/3.0,
            +M_PI, +M_PI/2.0, +3.0*M_PI/4.0, +M_PI, +M_PI/2.0, +M_PI, +M_PI/3.0, +M_PI/3.0, +M_PI/3.0;

        _angular_velocity_limit.resize(2, _joint_num);
        _angular_velocity_limit.setZero();

        _angular_acceleration_limit.resize(2, _joint_num);
        _angular_acceleration_limit.setZero();

        _jacobian_determinant_limit = 1.0e-6;
        */

        /* SCARA *//*
        _torque_control_enable = true;

        _chain_num = 1;
        _joint_num = 4;

        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
            1, 1, 1, 1;

        _joint_position_link.resize(3, (_joint_num + _chain_num));
        _joint_position_link <<
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0;

        _translation_axis.resize(3, _joint_num);
        _translation_axis <<
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 1;

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
            0, 0, 0, 0,
            0, 0, 0, 0,
            1, 1, 1, 0;

        if(_torque_control_enable)
        {
            _inertia.resize(10, _joint_num);
            _inertia.setConstant(0.0);
            _inertia.row(0).setConstant(1.0);
            _inertia.row(1).setConstant(1.0);
            _inertia.row(5).setConstant(1.0);
            _inertia.row(9).setConstant(1.0);
            _inertia.row(2).setConstant(0.1);
            _inertia.row(3).setConstant(0.1);
            _inertia.row(4).setConstant(0.1);
            _inertia.row(6).setConstant(0.1);
            _inertia.row(7).setConstant(0.1);
            _inertia.row(8).setConstant(0.1);

            _center_of_gravity_link.resize(3, _joint_num);
            _center_of_gravity_link.transpose() <<
                0, 0.5, 0,
                0, 0.5, 0,
                0, 0, 0,
                0, 0, 0;

            // _angle_torque_control_p_gain = 500.0;
            _angle_torque_control_p_gain = 50.0;
            // _angle_torque_control_d_gain = -1.0;

            _gravitational_acceleration = 9.8;
        }

        _angle_velocity_control_p_gain.resize(_joint_num, _joint_num);
        _angle_velocity_control_p_gain.setIdentity(_joint_num, _joint_num);

        _pose_velocity_control_p_gain = 1.0;
        */

        /* test *//*
        _torque_control_enable = true;

        _chain_num = 3;
        _joint_num = 10;

        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
        // 1  2  3  4  5  6  7  8  9 10
        1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 0, 1, 0,
        1, 1, 1, 1, 1, 1, 1, 0, 0, 1;

        _joint_position_link.resize(3, (_joint_num + _chain_num));
        _joint_position_link <<
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _translation_axis.resize(3, _joint_num);
        _translation_axis <<
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
        1, 0, 0, 1, 0, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0, 1, 0, 1, 0,
        0, 0, 1, 0, 0, 1, 0, 0, 0, 1;

        if(_torque_control_enable)
        {
            _inertia.resize(10, _joint_num);
            _inertia.setConstant(1.0);

            _center_of_gravity_link.resize(3, _joint_num);
            _center_of_gravity_link <<
            0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
            0  , 0  , 0  , 0  , 0  , 0  , 1  , 0.5, 0  , 0  ,
            0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  ;
            // _center_of_gravity_link = 0.5*_joint_position_link;

            _angle_torque_control_p_gain = 0.3;
            _angle_torque_control_d_gain = -0.5;

            _gravitational_acceleration = 9.8;
        }

        _angle_velocity_control_p_gain.resize(_joint_num, _joint_num);
        _angle_velocity_control_p_gain.setIdentity(_joint_num, _joint_num);

        _pose_velocity_control_p_gain = 1.0;
        */
    }

    // Change Model
    void Model::changeModel(const int &chain_num_,
                            const int &joint_num_,
                            const Eigen::Matrix<bool, -1, -1> &chain_mat_,
                            const Eigen::Matrix<double, 3, -1> &joint_position_link_,
                            const Eigen::Matrix<double, 3, -1> &translation_axis_,
                            const Eigen::Matrix<double, 3, -1> &rotation_axis_,
                            const Eigen::Matrix<double, -1, -1> &angle_velocity_control_p_gain_,
                            const double &pose_velocity_control_p_gain_)
    {
        // TODO: If any of the parameters cannot be changed, emergency stop change true.
        changeChainNum(chain_num_);
        changeJointNum(joint_num_);
        changeChainMatrix(chain_mat_);
        changeJointPositionLink(joint_position_link_);
        changeTranslationAxis(translation_axis_);
        changeRotationAxis(rotation_axis_);
        changeAngleVelocityControlPGain(angle_velocity_control_p_gain_);
        changePoseVelocityControlPGain(pose_velocity_control_p_gain_);
    }

    void Model::changeModel(const int &chain_num_,
                            const int &joint_num_,
                            const Eigen::Matrix<bool, -1, -1> &chain_mat_,
                            const Eigen::Matrix<double, 3, -1> &joint_position_link_,
                            const Eigen::Matrix<double, 3, -1> &translation_axis_,
                            const Eigen::Matrix<double, 3, -1> &rotation_axis_,
                            const Eigen::Matrix<double, 10, -1> &inertia_,
                            const Eigen::Matrix<double, 3, -1> &center_of_gravity_link_,
                            const Eigen::Matrix<double, -1, -1> &angle_velocity_control_p_gain_,
                            const double &pose_velocity_control_p_gain_)
    {
        // TODO: If any of the parameters cannot be changed, emergency stop change true.
        changeChainNum(chain_num_);
        changeJointNum(joint_num_);
        changeChainMatrix(chain_mat_);
        changeJointPositionLink(joint_position_link_);
        changeTranslationAxis(translation_axis_);
        changeRotationAxis(rotation_axis_);
        if(_torque_control_enable)
        {
            changeInertia(inertia_);
            changeCenterOfGravityLink(center_of_gravity_link_);
        }
        changeAngleVelocityControlPGain(angle_velocity_control_p_gain_);
        changePoseVelocityControlPGain(pose_velocity_control_p_gain_);
    }
    void Model::changeTorqueControlEnable(const bool &torque_control_enable_)
    {
        _torque_control_enable = torque_control_enable_;
    }

    void Model::changeChainNum(const int &chain_num_)
    {
        _chain_num = chain_num_;
    }

    void Model::changeJointNum(const int &joint_num_)
    {
        _joint_num = joint_num_;
    }

    void Model::changeBindingConditions(const int &binding_conditions_)
    {
        if(binding_conditions_ <= 0 || 6 < binding_conditions_)
        {
            std::cout << "Binding Conditions is wrong." << std::endl;
        }

        _binding_conditions = binding_conditions_;
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

    void Model::changeInertia(const Eigen::Matrix<double, 10, -1> &inertia_)
    {
        if(inertia_.cols() == _joint_num)
        {
            _inertia.resize(10, _joint_num);
            _inertia = inertia_;
        }
        else
        {
            std::cout << "Matrix(inertia_) Size do not match Joint Number." << std::endl;
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

    void Model::changeAngleVelocityControlPGain(const Eigen::Matrix<double, -1, -1> &angle_velocity_control_p_gain_)
    {
        if(angle_velocity_control_p_gain_.rows() == _joint_num && angle_velocity_control_p_gain_.cols() == _joint_num)
        {
            _angle_velocity_control_p_gain.resize(_joint_num, _joint_num);
            _angle_velocity_control_p_gain = angle_velocity_control_p_gain_;
        }
        else
        {
            std::cout << "Matrix(angle_velocity_control_p_gain_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changePoseVelocityControlPGain(const double &pose_velocity_control_p_gain_)
    {
        _pose_velocity_control_p_gain = pose_velocity_control_p_gain_;
    }

    void Model::changeAngleTorqueControlPGain(const double &angle_torque_control_p_gain_)
    {
        _angle_torque_control_p_gain = angle_torque_control_p_gain_;
    }

    void Model::changeAngleTorqueControlDGain(const double &angle_torque_control_d_gain_)
    {
        _angle_torque_control_d_gain = angle_torque_control_d_gain_;
    }

    void Model::changeGravitationalAcceleration(const double &gravitational_acceleration_)
    {
        _gravitational_acceleration = gravitational_acceleration_;
    }

    void Model::changeAngleLimit(const Eigen::Matrix<double, 2, -1> &angle_limit_)
    {
        if(angle_limit_.cols() != _joint_num)
        {
            std::cout << "Matrix(angle_limit_) Size do not match Joint Number." << std::endl;
            return;
        }

        _angle_limit.resize(2, _joint_num);

        for(int joint = 0; joint < _joint_num; joint++)
        {
            _angle_limit(0,joint) = angle_limit_(0,joint);
            _angle_limit(1,joint) = angle_limit_(1,joint);
        }
    }

    void Model::changeAngularVelocityLimit(const Eigen::Matrix<double, 2, -1> &angular_velocity_limit_)
    {
        if(angular_velocity_limit_.cols() != _joint_num)
        {
            std::cout << "Matrix(angular_velocity_limit_) Size do not match Joint Number." << std::endl;
            return;
        }

        _angular_velocity_limit.resize(2, _joint_num);

        for(int joint = 0; joint < _joint_num; joint++)
        {
            _angular_velocity_limit(0,joint) = angular_velocity_limit_(0,joint);
            _angular_velocity_limit(1,joint) = angular_velocity_limit_(1,joint);
        }
    }

    void Model::changeAngularAccelerationLimit(const Eigen::Matrix<double, 2, -1> &angular_acceleration_limit_)
    {
        if(angular_acceleration_limit_.cols() != _joint_num)
        {
            std::cout << "Matrix(angular_acceleration_limit_) Size do not match Joint Number." << std::endl;
            return;
        }

        _angular_acceleration_limit.resize(2, _joint_num);

        for(int joint = 0; joint < _joint_num; joint++)
        {
            _angular_acceleration_limit(0,joint) = angular_acceleration_limit_(0,joint);
            _angular_acceleration_limit(1,joint) = angular_acceleration_limit_(1,joint);
        }
    }

    void Model::changeJacobianDeterminantLimit(const double &jacobian_determinant_limit_)
    {
        _jacobian_determinant_limit = jacobian_determinant_limit_;
    }

    // Parameter Getters
    bool Model::getTorqueControlEnable()
    {
        return _torque_control_enable;
    }

    int Model::getJointNum()
    {
        return _joint_num;
    }

    int Model::getChainNum()
    {
        return _chain_num;
    }

    int Model::getBindingConditions()
    {
        return _binding_conditions;
    }

    Eigen::Matrix<bool, -1, -1> Model::getChainMat()
    {
        return _chain_mat;
    }

    Eigen::Matrix<double, -1, 6> Model::getBindingConditionsMatrix()
    {
        _binding_conditions_matrix.resize(_binding_conditions, 6);
        _binding_conditions_matrix.setZero();

        if(_binding_conditions == 1)
        {
            _binding_conditions_matrix(0,2) = 1;
        }
        else if(_binding_conditions == 3)
        {
            _binding_conditions_matrix(0,0) = 1;
            _binding_conditions_matrix(1,1) = 1;
            _binding_conditions_matrix(2,2) = 1;
        }
        else if(_binding_conditions == 4)
        {
            _binding_conditions_matrix(0,0) = 1;
            _binding_conditions_matrix(1,1) = 1;
            _binding_conditions_matrix(2,2) = 1;
            _binding_conditions_matrix(3,5) = 1;
        }
        else if(_binding_conditions == 6)
        {
            _binding_conditions_matrix.setIdentity();
        }

        return _binding_conditions_matrix;
    }

    Eigen::Matrix<double, 3, 1> Model::getJointPositionLink(const int &joint_)
    {
        if(joint_ < 0 || (_joint_num + _chain_num) <= joint_) return Eigen::Matrix<double, 3, 1>::Zero();
        return _joint_position_link.col(joint_);
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

    Eigen::Matrix<double, 6, 6> Model::getInertia(const int &joint_)
    {
        if(joint_ < 0 || _joint_num <= joint_) return Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> inertia_;
        inertia_.setZero();
        inertia_(0, 0) = _inertia(0, joint_);
        inertia_(1, 1) = _inertia(0, joint_);
        inertia_(2, 2) = _inertia(0, joint_);
        inertia_.block(3, 3, 3, 3) <<
            _inertia(1, joint_), _inertia(2, joint_), _inertia(3, joint_),
            _inertia(4, joint_), _inertia(5, joint_), _inertia(6, joint_),
            _inertia(7, joint_), _inertia(8, joint_), _inertia(9, joint_);
        return inertia_;
    }

    Eigen::Matrix<double, 3, 1> Model::getCenterOfGravityLink(const int &joint_)
    {
        if(joint_ < 0 || _joint_num <= joint_) return Eigen::Matrix<double, 3, 1>::Zero();
        return _center_of_gravity_link.col(joint_);
    }

    Eigen::Matrix<double, -1, -1> Model::getAngleVelocityControlPGain()
    {
        return _angle_velocity_control_p_gain;
    }

    double Model::getPoseVelocityControlPGain()
    {
        return _pose_velocity_control_p_gain;
    }

    double Model::getAngleTorqueControlPGain()
    {
        return _angle_torque_control_p_gain;
    }

    double Model::getAngleTorqueControlDGain()
    {
        return _angle_torque_control_d_gain;
    }

    double Model::getGravitationalAcceleration()
    {
        return _gravitational_acceleration;
    }

    Eigen::Matrix<double, 2, -1> Model::getAngleLimit()
    {
        for(int joint = 0; joint < _joint_num; joint++)
        {
            if(_angle_limit(0,joint) == _angle_limit(1,joint))
            {
                _angle_limit(0,joint) = -M_PI/2.0;
                _angle_limit(1,joint) = +M_PI/2.0;
            }
        }

        return _angle_limit;
    }

    Eigen::Matrix<double, 2, -1> Model::getAngularVelocityLimit()
    {
        for(int joint = 0; joint < _joint_num; joint++)
        {
            if(_angular_velocity_limit(0,joint) == _angular_velocity_limit(1,joint))
            {
                _angular_velocity_limit(0,joint) = -M_PI;
                _angular_velocity_limit(1,joint) = +M_PI;
            }
        }

        return _angular_velocity_limit;
    }

    Eigen::Matrix<double, 2, -1> Model::getAngularAccelerationLimit()
    {
        for(int joint = 0; joint < _joint_num; joint++)
        {
            if(_angular_acceleration_limit(0,joint) == _angular_acceleration_limit(1,joint))
            {
                _angular_acceleration_limit(0,joint) = -M_PI;
                _angular_acceleration_limit(1,joint) = +M_PI;
            }
        }

        return _angular_acceleration_limit;
    }

    double Model::getJacobianDeterminantLimit()
    {
        return _jacobian_determinant_limit;
    }
}