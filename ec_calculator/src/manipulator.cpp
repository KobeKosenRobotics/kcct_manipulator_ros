#include "ec_calculator/manipulator.h"
#include "ec_calculator/eigenUtility.h"
#include "ec_calculator/differential_integral.h"

namespace ec_calculator
{
    // Initialize
    void Manipulator::init(Model* model_)
    {
        _model = model_;

        _torque_enable = _model->getTorqueControlEnable();

        _JOINT_NUM = _model->getJointNum();
        _CHAIN_NUM = _model->getChainNum();
        _BINDING_CONDITIONS = _model->getBindingConditions();
        _joints.clear();
        _joints.resize(_JOINT_NUM+_CHAIN_NUM);

        _angle.resize(_JOINT_NUM, 1);
        _target_angle.resize(_JOINT_NUM, 1);
        _angular_velocity.resize(_JOINT_NUM, 1);
        _target_angular_velocity.resize(_JOINT_NUM, 1);
        _angular_acceleration.resize(_JOINT_NUM, 1);
        _target_angular_acceleration.resize(_JOINT_NUM, 1);
        _torque.resize(_JOINT_NUM, 1);
        _target_torque.resize(_JOINT_NUM, 1);
        clearParameters();

        _ik_interpolation.resize(1);
        _start_joint_index.resize(1);
        _end_joint_index.resize(1);
        _target_pose.resize(1);
        _jacobian_block.resize(1);

        _Mf.resize(_JOINT_NUM, _JOINT_NUM);
        _jacobian_g.resize(_JOINT_NUM);
        _Cf.resize(_JOINT_NUM, _JOINT_NUM);
        _dMab_dThc.resize(_JOINT_NUM*_JOINT_NUM*_JOINT_NUM);
        _d_adj_inv_gab_d_thc.resize(_JOINT_NUM*_JOINT_NUM*_JOINT_NUM);
        _Nf.resize(_JOINT_NUM, 1);

        for(int index = 0; index < _JOINT_NUM; index++)
        {
            _joints[index].init(index, "Joint" + std::to_string(index));
        }
        for(int index = _JOINT_NUM; index < (_JOINT_NUM + _CHAIN_NUM); index++)
        {
            _joints[index].init(index, "Tool" + std::to_string(index-_JOINT_NUM));
        }

        setChainMatrix(_model->getChainMat());
        setBindingConditionsMatrix(_model->getBindingConditionsMatrix());

        setJointParameters();

        setAngleVelocityControlPGain(_model->getAngleVelocityControlPGain());
        setPoseVelocityControlPGain(_model->getPoseVelocityControlPGain());
        setAngleTorqueControlPGain(_model->getAngleTorqueControlPGain());
        setAngleTorqueControlDGain(_model->getAngleTorqueControlDGain());

        setGravitationalAcceleration(_model->getGravitationalAcceleration());

        setAngleLimit(_model->getAngleLimit());
        setAngularVelocityLimit(_model->getAngularVelocityLimit());
        setAngularAccelerationLimit(_model->getAngularAccelerationLimit());
        setJacobianDeterminantLimit(_model->getJacobianDeterminantLimit());

        _is_first_during_time_measurement = true;
    }

    void Manipulator::clearParameters()
    {
        _angle.setZero();
        _target_angle.setZero();
        _target_angle_interpolation.setLinearVelocity(0.3);
        _target_angle_interpolation.setPoint(_angle, _target_angle);
        _angular_velocity.setZero();
        _target_angular_velocity.setZero();
        _angular_acceleration.setZero();
        _target_angular_acceleration.setZero();
        _torque.setZero();
        _target_torque.setZero();
    }

    bool Manipulator::setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_matrix_)
    {
        std::cout << "\nSetChainMatrix:" << std::endl;
        for(int chain = 0; chain < _CHAIN_NUM; chain++)
        {
            int parent = -1;
            for(int child = 0; child < _JOINT_NUM; child++)
            {
                if(!chain_matrix_(chain, child)) continue;
                if(parent != -1)
                {
                    _joints[child].setParent(_joints[parent]);
                    if(_joints[parent].addChild(_joints[child]))
                    {
                        /* Debug */
                        std::cout << _joints[child].getName() << " is set as child of " << _joints[parent].getName() << std::endl;
                    }
                }
                parent = child;
            }
        }

        int tool_ = _JOINT_NUM;
        clearTipIndex();
        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            if(_joints[joint].isTipJoint())
            {
                _joints[tool_].setParent(_joints[joint]);
                if(_joints[joint].addChild(_joints[tool_]))
                {
                    /* Debug */
                    std::cout << _joints[tool_].getName() << " is set as child of " << _joints[joint].getName() << std::endl;
                    setTipIndex(tool_);
                    tool_++;
                }
            }
        }
        return true;
    }

    void Manipulator::setBindingConditionsMatrix(const Eigen::Matrix<double, -1, 6> &binding_conditions_matrix_)
    {
        _binding_conditions_matrix.resize(_BINDING_CONDITIONS, 6);
        _binding_conditions_matrix = binding_conditions_matrix_;
    }

    void Manipulator::setJointParameters()
    {
        for(int joint = 0; joint < (_JOINT_NUM + _CHAIN_NUM); joint++)
        {
            _joints[joint].setParameters(_model);
        }
    }

    void Manipulator::setTipIndex(const int &tip_index_)
    {
        _tip_index.push_back(tip_index_);
    }

    void Manipulator::clearTipIndex()
    {
        _tip_index.clear();
    }

    void Manipulator::setAngleVelocityControlPGain(const Eigen::Matrix<double, -1, -1> &angle_velocity_control_p_gain_)
    {
        _angle_velocity_control_p_gain = angle_velocity_control_p_gain_;
    }

    void Manipulator::setPoseVelocityControlPGain(const double &pose_velocity_control_p_gain_)
    {
        _pose_velocity_control_p_gain = pose_velocity_control_p_gain_;
    }

    void Manipulator::setAngleTorqueControlPGain(const double &angle_torque_control_p_gain_)
    {
        _angle_torque_control_p_gain = angle_torque_control_p_gain_;
    }

    void Manipulator::setAngleTorqueControlDGain(const double &angle_torque_control_d_gain_)
    {
        _angle_torque_control_d_gain = angle_torque_control_d_gain_;
    }

    void Manipulator::setGravitationalAcceleration(const double &gravitational_acceleration_)
    {
        _gravitational_acceleration = gravitational_acceleration_;
    }

    void Manipulator::setAngleLimit(const Eigen::Matrix<double, 2, -1> &angle_limit_)
    {
        _angle_limit.resize(2, _JOINT_NUM);
        _angle_limit = angle_limit_;
    }

    void Manipulator::setAngularVelocityLimit(const Eigen::Matrix<double, 2, -1> &angular_velocity_limit_)
    {
        _angular_velocity_limit.resize(2, _JOINT_NUM);
        _angular_velocity_limit = angular_velocity_limit_;
    }

    void Manipulator::setAngularAccelerationLimit(const Eigen::Matrix<double, 2, -1> &angular_acceleration_limit_)
    {
        _angular_acceleration_limit.resize(2, _JOINT_NUM);
        _angular_acceleration_limit = angular_acceleration_limit_;
    }

    void Manipulator::setJacobianDeterminantLimit(const double &jacobian_determinant_limit_)
    {
        _jacobian_determinant_limit = jacobian_determinant_limit_;
    }

    // Properties
    int Manipulator::getChainNum()
    {
        return _CHAIN_NUM;
    }

    int Manipulator::getJointNum()
    {
        return _JOINT_NUM;
    }

    std::string Manipulator::getJointName(const int &joint_index_)
    {
        if(joint_index_ < 0 || (_JOINT_NUM + _CHAIN_NUM) <= joint_index_) return "getJointName(): Exception Handling";

        return _joints[joint_index_].getName();
    }

    std::string Manipulator::getJointParentName(const int &joint_index_)
    {
        if(joint_index_ < 0 || (_JOINT_NUM + _CHAIN_NUM) <= joint_index_) return "getJointParentName(): Exception Handling";

        return _joints[joint_index_].getParentName();
    }

    int Manipulator::getInverseKinematicsNum()
    {
        return _ik_index;
    }

    bool Manipulator::getPolygonEnable()
    {
        return _polygon_enable;
    }

    bool Manipulator::getSimulationEnable()
    {
        return _simulation_enable;
    }

    bool Manipulator::getTorqueEnable()
    {
        return _torque_enable;
    }

    // Visualize
    double Manipulator::getVisualData(const int &joint_index_, const int &data_index_)
    {
        if(joint_index_ < 0 || (_JOINT_NUM + _CHAIN_NUM) <= joint_index_) return 0.0;

        return _joints[joint_index_].getVisualData(data_index_);
    }

    // Forward Kinematics
    void Manipulator::updateAngle(const Eigen::Matrix<double, -1, 1> &angle_)
    {
        _angle = angle_;
        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            _joints[joint].updateTheta(_angle(joint, 0));
        }
    }

    void Manipulator::updateAngularVelocity(const Eigen::Matrix<double, -1, 1> &angular_velocity_)
    {
        _angular_velocity = angular_velocity_;
    }

    void Manipulator::updateAngularAcceleration(const Eigen::Matrix<double, -1, 1> &angular_velocity_)
    {
        _angular_acceleration = _angular_acc_diff.differential(angular_velocity_);
    }

    void Manipulator::updateTorque(const Eigen::Matrix<double, -1, 1> &torque_)
    {
        _torque = torque_;
    }

    Eigen::Matrix<double, 6, 1> Manipulator::getPose(const int &joint_index_)
    {
        return EigenUtility.getPose(_joints[joint_index_].getGstTheta());
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getAngle()
    {
        return _angle;
    }

    // AC or IK
    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocity()
    {
        if(_torque_enable) return _target_angular_velocity.setZero();

        // Emergency Stop
        if(_emergency_stop)
        {
            _target_angular_velocity.setZero();
            angularVelocity2Angle(_target_angular_velocity);
            return _target_angular_velocity;
        }

        // Calculation
        if(_ik_enable)
        {
            getAngularVelocityByEC();
        }
        else
        {
            getAngularVelocityByAngle();
        }

        // Angle Limit
        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            if(_angle_limit(0,joint) < _angle(joint,0) && _angle(joint,0) < _angle_limit(1,joint)) continue;

            if(_angle(joint,0) <= _angle_limit(0,joint) && 0.0 <= _target_angular_velocity(joint,0)) continue;

            else if(_angle_limit(1,joint) <= _angle(joint,0) && _target_angular_velocity(joint,0) <= 0.0) continue;

            _target_angular_velocity(joint,0) = 0.0;
        }

        // Angular Velocity Limit
        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            _target_angular_velocity(joint,0) = std::min(std::max(_target_angular_velocity(joint,0), _angular_velocity_limit(0,joint)), _angular_velocity_limit(1,joint));
        }

        // Angular Acceleration Limit
        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            if(!_ik_enable)
            {
                if(fabs(_angular_velocity(joint,0)) < 0.10*_angular_velocity_limit(1,joint)) continue;
            }

            if(fabs(_angular_velocity(joint,0)) < 0.05*_angular_velocity_limit(1,joint)) continue;

            if(_angular_acceleration_limit(0,joint) < _angular_acceleration(joint,0) && _angular_acceleration(joint,0) < _angular_acceleration_limit(1,joint)) continue;
            _target_angular_velocity(joint,0) *= 0.5;

            _emergency_stop = true;
            std::cout << "Angular Acceleration Limit" << std::endl;
        }

        // Motor Limit
        if(!_motor_enable)
        {
            angularVelocity2Angle(_target_angular_velocity);
            return EigenUtility.getZeros(_JOINT_NUM, 1);
        }

        return _target_angular_velocity;
    }

    // Angle Command
    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByAngle(const Eigen::Matrix<double, -1, 1> &target_angle_)
    {
        _target_angular_velocity = _angle_velocity_control_p_gain * (target_angle_ - _angle);

        return _target_angular_velocity;
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByAngle()
    {
        // _target_angular_velocity = _target_angle_interpolation.getDSinInterpolation() + _angle_velocity_control_p_gain * (_target_angle_interpolation.getSinInterpolation() - _angle);
        _target_angular_velocity = _angle_velocity_control_p_gain * (_target_angle_interpolation.getSinInterpolation() - _angle);

        return _target_angular_velocity;
    }

    // Inverse Kinematics
    void Manipulator::setJointPose(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> &target_pose_)
    {
        _start_joint_index[_ik_index] = start_joint_index_;
        _end_joint_index[_ik_index] = end_joint_index_;
        if(_target_pose[_ik_index] != target_pose_)
        {
            _target_pose[_ik_index] = target_pose_;
            _ik_interpolation[_ik_index].setPoint(_binding_conditions_matrix*getPose(_end_joint_index[_ik_index]), _binding_conditions_matrix*_target_pose[_ik_index]);
        }
        _ik_index++;
    }

    void Manipulator::clearJointPose()
    {
        _ik_index = 0;
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByEC()
    {
        if(_ik_index == 0) return _target_angular_velocity;

        _error_all.resize(_BINDING_CONDITIONS*_ik_index, 1);
        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            // _error_all.block(_BINDING_CONDITIONS*ik_index, 0, _BINDING_CONDITIONS, 1) = (_ik_interpolation[ik_index].getDSinInterpolation() + _pose_velocity_control_p_gain*(_ik_interpolation[ik_index].getSinInterpolation() - (_binding_conditions_matrix*getPose(_end_joint_index[ik_index]))));
            _error_all.block(_BINDING_CONDITIONS*ik_index, 0, _BINDING_CONDITIONS, 1) = (_pose_velocity_control_p_gain*(_ik_interpolation[ik_index].getSinInterpolation() - (_binding_conditions_matrix*getPose(_end_joint_index[ik_index]))));
        }

        getJacobian();

        // Singular Configuration
        double jacobian_determinant_ = fabs((_jacobian*_jacobian.transpose()).determinant());
        if(jacobian_determinant_ < _jacobian_determinant_limit)
        {
            _target_angular_velocity *= 0.5;
            _emergency_stop = true;
            std::cout << "near Singular Configuration" << std::endl;

            return _target_angular_velocity;
        }

        _target_angular_velocity = EigenUtility.getPseudoInverseMatrix(_jacobian) * _error_all;

        return _target_angular_velocity;
    }

    Eigen::Matrix<double, -1, -1> Manipulator::getJacobian()
    {
        _jacobian.resize(_BINDING_CONDITIONS*_ik_index, _JOINT_NUM);

        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _jacobian.block(_BINDING_CONDITIONS*ik_index, 0, _BINDING_CONDITIONS, _JOINT_NUM) = _binding_conditions_matrix*getJacobianBlock(ik_index);
        }

        return _jacobian;
    }

    Eigen::Matrix<double, 6, -1> Manipulator::getJacobianBlock(const int &ik_index_)
    {
        _jacobian_block[ik_index_].resize(6, _JOINT_NUM);
        _jacobian_block[ik_index_].setZero();

        for(int ik_index = _start_joint_index[ik_index_]; ik_index < std::min(_end_joint_index[ik_index_], _JOINT_NUM); ik_index++)
        {
            _jacobian_block[ik_index_].block(0, ik_index, 6, 1) = _joints[_end_joint_index[ik_index_]].getXiDagger(ik_index);
        }

        _jacobian_block[ik_index_] = (EigenUtility.getTransformationMatrix(_joints[_end_joint_index[ik_index_]].getGstTheta())).inverse() * _jacobian_block[ik_index_];

        return _jacobian_block[ik_index_];
    }

    // Torque Control
    Eigen::Matrix<double, -1, 1> Manipulator::getTorqueByAngle()
    {
        if(!_torque_enable) return _target_torque.setZero();

        // TODO: integrate into getTorque()
        if(_emergency_stop)
        {
            _target_torque.setZero();
            torque2Angle(_target_torque);
            return _target_torque;
        }

        getMf();
        getCf();
        getNf();

        /* Gain Example *//*
        Eigen::Matrix<double, 6, 6> gain_matrix_;
        gain_matrix_.setZero();
        gain_matrix_(0,0) = 1000.0;
        gain_matrix_(1,1) = 10.0;
        gain_matrix_(2,2) = 15.0;
        gain_matrix_(3,3) = 800.0;
        gain_matrix_(4,4) = 50; // _angle_torque_control_p_gain;

        _target_torque = _Mf * (gain_matrix_ * (_target_angle_interpolation.getSinInterpolation() - _angle)) + _Cf*_angular_velocity + _Nf;
        */

        _target_torque = _Mf * (_target_angle_interpolation.getDDSinInterpolation() + _angle_torque_control_d_gain*(_target_angle_interpolation.getDSinInterpolation() - _angular_velocity) + _angle_torque_control_p_gain * (_target_angle_interpolation.getSinInterpolation() - _angle)) + _Cf*_angular_velocity + _Nf;

        // TODO: integrate into getTorque()
        if(!_motor_enable)
        {
            torque2Angle(_target_torque);
            return EigenUtility.getZeros(_JOINT_NUM, 1);
        }

        return _target_torque;
    }

    Eigen::Matrix<double, -1, -1> Manipulator::getMf()
    {
        _Mf.setZero();

        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            getJacobianG(joint);
            _Mf += (_jacobian_g[joint].transpose() * _joints[joint].getI() * _jacobian_g[joint]);
        }

        return _Mf;
    }

    Eigen::Matrix<double, 6, -1> Manipulator::getJacobianG(const int &joint_index_)
    {
        _jacobian_g[joint_index_].resize(6, _JOINT_NUM);
        _jacobian_g[joint_index_].setZero();

        for(int joint = 0; joint <= joint_index_; joint++)
        {
            _jacobian_g[joint_index_].block(0, joint, 6, 1) = _joints[joint_index_].getXiDaggerG(joint);
        }

        _jacobian_g[joint_index_] = EigenUtility.getTransformationMatrix(_joints[joint_index_].getGsrTheta()) * _jacobian_g[joint_index_];

        return _jacobian_g[joint_index_];
    }

    Eigen::Matrix<double, -1, -1> Manipulator::getCf()
    {
        _Cf.setZero();

        // TODO: make function for pre-calculation
        _was_dMab_dThc_calculated = false;
        _was_d_adj_inv_gab_d_thc_calculated = false;

        for(int i = 0; i < _JOINT_NUM; i++)
        {
            for(int j = 0; j < _JOINT_NUM; j++)
            {
                for(int k = 0; k < _JOINT_NUM; k++)
                {
                    get_dAdjointInverseGab_dThc(i, j, k);
                }
            }
        }
        _was_d_adj_inv_gab_d_thc_calculated = true;

        for(int i = 0; i < _JOINT_NUM; i++)
        {
            for(int j = 0; j < _JOINT_NUM; j++)
            {
                for(int k = 0; k < _JOINT_NUM; k++)
                {
                    get_dMab_dThc(i, j, k);
                }
            }
        }

        _was_dMab_dThc_calculated = true;

        for(int joint_i = 0; joint_i < _JOINT_NUM; joint_i++)
        {
            for(int joint_j = 0; joint_j < _JOINT_NUM; joint_j++)
            {
                _Cf(joint_i, joint_j) = getCfBlock(joint_i, joint_j);
            }
        }

        return _Cf;
    }

    double Manipulator::getCfBlock(const int &i_, const int &j_)
    {
        double Cf_block_ = 0.0;

        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            Cf_block_ += (get_dMab_dThc(i_, j_, joint) + get_dMab_dThc(i_, joint, j_) - get_dMab_dThc(joint, j_, i_)) * _angular_velocity(joint, 0);
        }

        Cf_block_ *= 0.5;

        return Cf_block_;
    }

    double Manipulator::get_dMab_dThc(const int &a_, const int &b_, const int &c_)
    {
        int index_ = a_*_JOINT_NUM*_JOINT_NUM + b_*_JOINT_NUM + c_;

        if(_was_dMab_dThc_calculated) return _dMab_dThc[index_];

        _dMab_dThc[index_] = 0.0;

        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            // TODO: divide the calculation
            _dMab_dThc[index_] += ((_joints[a_].getXi()).transpose() * ((get_dAdjointInverseGab_dThc(a_, joint, c_)).transpose() * _joints[joint].getI() * EigenUtility.adjointInverse(_joints[joint].getParentGsrTheta(b_)) + EigenUtility.adjointInverse(_joints[joint].getParentGsrTheta(a_)).transpose() * _joints[joint].getI() * get_dAdjointInverseGab_dThc(b_, joint, c_)) * _joints[b_].getXi());
        }

        return _dMab_dThc[index_];
    }

    Eigen::Matrix<double, 6, 6> Manipulator::get_dAdjointInverseGab_dThc(const int &a_, const int &b_, const int &c_)
    {
        // TODO: write easy to read
        int index_ = a_*_JOINT_NUM*_JOINT_NUM + b_*_JOINT_NUM + c_;

        if(_was_d_adj_inv_gab_d_thc_calculated) return _d_adj_inv_gab_d_thc[index_];

        _d_adj_inv_gab_d_thc[index_].setZero();

        if ((c_ < a_) || (b_ < c_)) return _d_adj_inv_gab_d_thc[index_];

        Eigen::Matrix<double, 4, 4> g_mat_, d_gab_mat_d_thc_;
        g_mat_ = _joints[b_].getParentGsrTheta(a_);
        d_gab_mat_d_thc_ = _joints[b_].get_dGsr_dTh(a_, c_);

        Eigen::Matrix<double, 3, 3> rot_, d_rot_;
        Eigen::Matrix<double, 3, 1> pos_, d_pos_;
        rot_ = g_mat_.block(0, 0, 3, 3);
        d_rot_ = d_gab_mat_d_thc_.block(0, 0, 3, 3);
        pos_ = g_mat_.block(0, 3, 3, 1);
        d_pos_ = d_gab_mat_d_thc_.block(0, 3, 3, 1);

        _d_adj_inv_gab_d_thc[index_] <<
            d_rot_.transpose(), - d_rot_.transpose() * EigenUtility.hat(pos_) - rot_.transpose() * EigenUtility.hat(d_pos_),
            Eigen::Matrix<double, 3, 3>::Zero(), d_rot_.transpose();

        return _d_adj_inv_gab_d_thc[index_];
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getNf()
    {
        _Nf.setZero();

        for(int joint_i = 0; joint_i < _JOINT_NUM; joint_i++)
        {
            for(int chain = 0; chain < _CHAIN_NUM; chain++)
            {
                for(int joint_j = 0; joint_j < _JOINT_NUM; joint_j++)
                {
                    _Nf(joint_i, 0) += _joints[joint_j].getI()(0, 0)*_gravitational_acceleration*_joints[joint_j].get_dGsr_dTh(0, joint_i)(2, 3);
                }
            }
        }

        return _Nf;
    }

    // Angular Velocity to Angle (for Visualization)
    Eigen::Matrix<double, -1, 1> Manipulator::angularVelocity2Angle(const Eigen::Matrix<double, -1, 1> &angular_velocity_)
    {
        _angular_velocity = angular_velocity_;
        _angular_acceleration = _angular_acc_diff.differential(_angular_velocity);

        if(_is_first_during_time_measurement)
        {
            _is_first_during_time_measurement = false;
            _angle.setZero();
            _start_time = std::chrono::system_clock::now();
        }

        _end_time = std::chrono::system_clock::now();
        _during_time = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_end_time-_start_time).count();
        _angle += (_during_time * _angular_velocity);
        _start_time = _end_time;

        updateAngle(_angle);

        return _angle;
    }

    Eigen::Matrix<double, -1, 1> Manipulator::torque2Angle(const Eigen::Matrix<double, -1, 1> &torque_)
    {
        _torque = torque_;

        _angular_acceleration = EigenUtility.getPseudoInverseMatrix(_Mf) * (_torque - (_Cf * _angular_velocity) - _Nf);

        if(_is_first_during_time_measurement)
        {
            _is_first_during_time_measurement = false;
            _angle.setZero();
            _start_time = std::chrono::system_clock::now();
        }

        _end_time = std::chrono::system_clock::now();
        _during_time = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(_end_time-_start_time).count();
        _start_time = _end_time;

        _angular_velocity += (_during_time * _angular_acceleration);
        _angle += (_during_time * _angular_velocity);

        updateAngle(_angle);

        return _angle;
    }

    // Subscriber
    void Manipulator::setIKEnable(const bool &ik_enable_)
    {
        if(_ik_enable == ik_enable_) return;

        _ik_enable = ik_enable_;

        if(_ik_enable)
        {
            for(int ik_index = 0; ik_index < _ik_index; ik_index++)
            {
                _ik_interpolation[ik_index].setPoint(_binding_conditions_matrix*getPose(_end_joint_index[ik_index]), _binding_conditions_matrix*_target_pose[ik_index]);
            }
        }
        else
        {
            _target_angle_interpolation.setPoint(_angle, _target_angle);
        }
    }

    void Manipulator::setEmergencyStop(const bool &emergency_stop_)
    {
        if(_emergency_stop == emergency_stop_) return;

        _emergency_stop = emergency_stop_;

        if(_emergency_stop) return;

        _target_angle_interpolation.setPoint(_angle, _target_angle);

        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _ik_interpolation[ik_index].setPoint(_binding_conditions_matrix*getPose(_end_joint_index[ik_index]), _binding_conditions_matrix*_target_pose[ik_index]);
        }
    }

    void Manipulator::setMotorEnable(const bool &motor_enable_)
    {
        if(_motor_enable == motor_enable_) return;

        _motor_enable = motor_enable_;

        _target_angle_interpolation.setPoint(_angle, _target_angle);

        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _ik_interpolation[ik_index].setPoint(_binding_conditions_matrix*getPose(_end_joint_index[ik_index]), _binding_conditions_matrix*_target_pose[ik_index]);
        }
    }

    bool Manipulator::getMotorEnable()
    {
        return _motor_enable;
    }

    void Manipulator::setPolygonEnable(const bool &polygon_enable_)
    {
        if(_polygon_enable == polygon_enable_) return;

        _polygon_enable = polygon_enable_;
    }

    void Manipulator::setSimulationEnable(const bool &simulation_enable_)
    {
        if(_simulation_enable == simulation_enable_) return;

        _simulation_enable = simulation_enable_;

        _target_angle_interpolation.setPoint(_angle, _target_angle);

        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _ik_interpolation[ik_index].setPoint(_binding_conditions_matrix*getPose(_end_joint_index[ik_index]), _binding_conditions_matrix*_target_pose[ik_index]);
        }
    }

    void Manipulator::setTorqueEnable(const bool &torque_enable_)
    {
        if(_torque_enable == torque_enable_) return;

        if(!_model->getTorqueControlEnable()) return;

        _torque_enable = torque_enable_;

        _target_angle_interpolation.setPoint(_angle, _target_angle);

        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _ik_interpolation[ik_index].setPoint(_binding_conditions_matrix*getPose(_end_joint_index[ik_index]), _binding_conditions_matrix*_target_pose[ik_index]);
        }
    }

    void Manipulator::setTargetAngle(const Eigen::Matrix<double, -1, 1> &target_angle_)
    {
        int size_ = target_angle_.rows();
        if(size_ > _JOINT_NUM) size_ = _JOINT_NUM;

        if(_target_angle.block(0, 0, size_, 1) == target_angle_.block(0, 0, size_, 1)) return;

        _target_angle.block(0, 0, size_, 1) = target_angle_.block(0, 0, size_, 1);
        _target_angle_interpolation.setPoint(_angle, _target_angle);
    }

    void Manipulator::setTargetPose(const Eigen::Matrix<double, -1, 1> &target_pose_)
    {
        // 8 = 2 + 6 = 1:start_joint + 1:end_joint + 3:position + 3:orientation
        int ik_index_num_ = target_pose_.rows()/8;
            _ik_interpolation.resize(ik_index_num_);
            _start_joint_index.resize(ik_index_num_);
            _end_joint_index.resize(ik_index_num_);
            _target_pose.resize(ik_index_num_);
            _jacobian_block.resize(ik_index_num_);
        clearJointPose();
        for(int ik_index = 0; ik_index < ik_index_num_; ik_index++)
        {
            setJointPose((int)(target_pose_(8*ik_index, 0)+0.5), (int)(target_pose_(8*ik_index+1, 0)+0.5), target_pose_.block(8*ik_index+2, 0, 6, 1));
        }
    }

    void Manipulator::setTargetPolygon(const Eigen::Matrix<double, -1, 1> &target_polygon_)
    {
        _ik_interpolation.resize(_CHAIN_NUM);
        _start_joint_index.resize(_CHAIN_NUM);
        _end_joint_index.resize(_CHAIN_NUM);
        _target_pose.resize(_CHAIN_NUM);
        _jacobian_block.resize(_CHAIN_NUM);

        clearJointPose();

        if(target_polygon_.size() < 7)
        {
            std::cout << "Target Polygon must be at least 7 dimensions." << std::endl;
        }

        Eigen::Matrix<double, 3, 1> base_vector_;
        base_vector_ << -1.0, 0.0, 0.0;
        Eigen::Matrix<double, 6, 1> polygon_pose_;

        for(int ik_index = 0; ik_index < _CHAIN_NUM; ik_index++)
        {
            polygon_pose_ = target_polygon_.block(0,0,6,1);
            polygon_pose_.block(0,0,3,1) += EigenUtility.getRotationMatrixZ(polygon_pose_(3,0))*EigenUtility.getRotationMatrixY(polygon_pose_(4,0))*EigenUtility.getRotationMatrixX(polygon_pose_(5,0))*(target_polygon_(6,0)*EigenUtility.getRotationMatrixZ(-2.0*ik_index*M_PI/(double)_CHAIN_NUM)*base_vector_);
            setJointPose(0, _tip_index[ik_index], polygon_pose_);
        }
    }

    // Debug
    void Manipulator::printTree()
    {
        std::cout << _joints[0].getChildrenList() << std::endl;
    }

    void Manipulator::print()
    {
        // std::cout << "Mf\n" << _Mf << std::endl << std::endl;
        // std::cout << "Cf\n" << _Cf << std::endl << std::endl;
        // std::cout << "Nf\n" << _Nf << std::endl << std::endl;

        // std::cout << "gain : " << _angle_torque_control_p_gain << std::endl;
        std::cout << "angle :" << std::endl << _angle << std::endl << std::endl;
        std::cout << "pose5 :" << std::endl << getPose(5) << std::endl << std::endl;
        // std::cout << "angular velocity :" << std::endl << _angular_velocity << std::endl << std::endl;
        // std::cout << "angular acceleration :" << std::endl << _angular_acceleration << std::endl << std::endl;

        // std::cout << "torque :" << std::endl << _torque << std::endl << std::endl;

        // std::cout << "cumulative time :" << std::endl << updateCumulativeTime() << std::endl << std::endl;

        // getNowTorque();

        // if(_motor_enable)
        // {
        //     /* Writing to a File */
        //     std::ofstream output_file("/home/amar/Documents/Tokuken/ConverterParameter/all_joint_move.csv", std::ios::app);

        //     output_file << _cumulative_time << ",";

        //     for(int j = 0; j < 6; j++)
        //     {
        //         output_file << _torque(j,0) << "," << _now_torque(j,0) << ",";
        //         // output_file << _target_torque(j,0) << "," << _torque(j,0) << ",";
        //     }

        //     output_file << std::endl;
        // }

        // get_SCARA();
    }

    Eigen::Matrix<double, 6, 1> Manipulator::getTargetPose(const int &ik_index_)
    {
        return _target_pose[ik_index_];
    }

    Eigen::Matrix<double, 6, 1> Manipulator::getMidPose(const int &ik_index_)
    {
        if(_ik_enable) return _binding_conditions_matrix.transpose()*_ik_interpolation[ik_index_].getSinInterpolation();

        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    void Manipulator::get_SCARA()
    {
        double a_, b_, c_, d_;
        a_ = 1.0 + 0.5*0.5*1.0 + 1.0*1.0*1.0 + 1.0*1.0*1.0 + 1.0*1.0*1.0;
        b_ = 1.0 + 1.0 + 1.0 + 1.0*1.0*1.0 + 1.0*1.0*1.0 + 1.0*0.5*0.5;
        c_ = 1.0*1.0*1.0 + 1.0*1.0*1.0 + 1.0*1.0*0.5;
        d_ = 1.0 + 1.0;

        Eigen::Matrix<double, 4, 4> Mf_SCARA_;
        Eigen::Matrix<double, 4, 4> Cf_SCARA_;
        Eigen::Matrix<double, 4, 1> Nf_SCARA_;

        Mf_SCARA_ <<
        a_+b_+2*c_*cos(_angle(1, 0)), b_+c_*cos(_angle(1, 0)), d_, 0.0,
        b_+c_*cos(_angle(1, 0)), b_, d_, 0.0,
        d_, d_, d_, 0.0,
        0.0, 0.0, 0.0, 1.0;

        Cf_SCARA_ <<
        -c_*sin(_angle(1, 0))*_angular_velocity(1, 0), -c_*sin(_angle(1, 0))*(_angular_velocity(0, 0)+_angular_velocity(1, 0)), 0.0, 0.0,
        c_*sin(_angle(1, 0))*_angular_velocity(0, 0), 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0;

        Nf_SCARA_ <<
        0.0, 0.0, 0.0, 1.0*9.8;

        std::cout << "Mf_SCARA\n" << Mf_SCARA_ << std::endl << std::endl;
        std::cout << "Cf_SCARA\n" << Cf_SCARA_ << std::endl << std::endl;
        std::cout << "Nf_SCARA\n" << Nf_SCARA_ << std::endl << std::endl;
    }

    Eigen::Matrix<double, 6, 1> Manipulator::getIdealTorque()
    {
        getMf();
        getCf();
        getNf();

        _angular_acceleration = _angular_acc_diff.differential(_angular_velocity);

        _ideal_torque = _Mf*_angular_acceleration + _Cf*_angular_velocity + _Nf;

        return _ideal_torque;
    }

    double Manipulator::updateCumulativeTime()
    {
        if(_is_first_cumulative_time_measurement)
        {
            _is_first_cumulative_time_measurement = false;
            _cumulative_time_start = std::chrono::system_clock::now();
        }

        return _cumulative_time = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-_cumulative_time_start).count();
    }
}