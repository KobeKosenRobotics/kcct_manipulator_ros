#include "ec_calculator/manipulator.h"
#include "ec_calculator/eigenUtility.h"

namespace ec_calculator
{
    // Initialize
    void Manipulator::init(Model* model_)
    {
        _model = model_;

        _JOINT_NUM = _model->getJointNum();
        _CHAIN_NUM = _model->getChainNum();
        _joints.clear();
        _joints.resize(_JOINT_NUM+_CHAIN_NUM);

        _angle.resize(_JOINT_NUM, 1);
        _target_angle.resize(_JOINT_NUM, 1);
        _angular_velocity.resize(_JOINT_NUM, 1);
        _target_angular_velocity.resize(_JOINT_NUM, 1);
        clearParameters();

        _interpolation.resize(1);
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

        setJointParameters();

        setAngle2AngularVelocityGain(_model->getAngle2AngularVelocityGain());

        setECGain(_model->getECGain());

        setGravitationalAcceleration(_model->getGravitationalAcceleration());

        _is_first_time_measurement = true;
    }

    void Manipulator::clearParameters()
    {
        _angle.setZero();
        _target_angle.setZero();
        _angular_velocity.setZero();
        _target_angular_velocity.setZero();
    }

    bool Manipulator::setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_mat)
    {
        std::cout << "\nSetChainMatrix:" << std::endl;
        for(int chain = 0; chain < _CHAIN_NUM; chain++)
        {
            int parent = -1;
            for(int child = 0; child < _JOINT_NUM; child++)
            {
                if(!chain_mat(chain, child)) continue;
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

    void Manipulator::setAngle2AngularVelocityGain(const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_)
    {
        _angle_2_angular_velocity_gain = angle_2_angular_velocity_gain_;
    }

    void Manipulator::setECGain(const double &ec_gain_)
    {
        _ec_gain = ec_gain_;
    }

    void Manipulator::setGravitationalAcceleration(const double &gravitational_acceleration_)
    {
        _gravitational_acceleration = gravitational_acceleration_;
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
        if(_ec_enable)
        {
            return getAngularVelocityByEC();
        }
        else
        {
            return getAngularVelocityByAngle();
        }
    }

    // Angle Command
    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByAngle(const Eigen::Matrix<double, -1, 1> &target_angle_)
    {
        _target_angular_velocity = _angle_2_angular_velocity_gain * (target_angle_ - _angle);
        return _target_angular_velocity;
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByAngle()
    {
        _target_angular_velocity = _angle_2_angular_velocity_gain * (_target_angle - _angle);
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
            _interpolation[_ik_index].setPoint(getPose(_end_joint_index[_ik_index]), _target_pose[_ik_index]);
        }
        _ik_index++;
    }

    void Manipulator::clearJointPose()
    {
        _ik_index = 0;
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByEC()
    {
        _error_all.resize(6*_ik_index, 1);
        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _error_all.block(6*ik_index, 0, 6, 1) = _interpolation[ik_index].getSigmoidInterpolation() - getPose(_end_joint_index[ik_index]);
        }
        _target_angular_velocity = EigenUtility.getPseudoInverseMatrix(getJacobian()) * _error_all;
        return _target_angular_velocity;
    }

    Eigen::Matrix<double, -1, -1> Manipulator::getJacobian()
    {
        _jacobian.resize(6*_ik_index, _JOINT_NUM);

        for(int ik_index = 0; ik_index < _ik_index; ik_index++)
        {
            _jacobian.block(6*ik_index, 0, 6, _JOINT_NUM) = getJacobianBlock(ik_index);
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

        _jacobian_block[ik_index_] = EigenUtility.getTransformationMatrix(_joints[_end_joint_index[ik_index_]].getGstTheta()) * _jacobian_block[ik_index_];

        return _jacobian_block[ik_index_];
    }

    // Torque Control
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
        double Cf_block_ = 0;

        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            Cf_block_ += (0.5 * (get_dMab_dThc(i_, j_, joint) + get_dMab_dThc(i_, joint, j_) - get_dMab_dThc(joint, j_, i_)));
        }

        return Cf_block_;
    }

    double Manipulator::get_dMab_dThc(const int &a_, const int &b_, const int &c_)
    {
        int index_ = a_*_JOINT_NUM*_JOINT_NUM + b_*_JOINT_NUM + c_;

        if(_was_dMab_dThc_calculated) return _dMab_dThc[index_];

        _dMab_dThc[index_] = 0;

        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            _dMab_dThc[index_] += ((_joints[a_].getXi()).transpose() * ((get_dAdjointInverseGab_dThc(a_, joint, c_)).transpose() * _joints[joint].getI() * EigenUtility.adjointInverse(_joints[joint].getParentGsrTheta(b_)) + EigenUtility.adjointInverse(_joints[joint].getParentGsrTheta(a_)) * _joints[joint].getI() * get_dAdjointInverseGab_dThc(b_, joint, c_)) * _joints[b_].getXi());
        }

        return _dMab_dThc[index_];
    }

    Eigen::Matrix<double, 6, 6> Manipulator::get_dAdjointInverseGab_dThc(const int &a_, const int &b_, const int &c_)
    {
        // TODO: write easy to read
        int index_ = a_*_JOINT_NUM*_JOINT_NUM + b_*_JOINT_NUM + c_;

        if(_was_d_adj_inv_gab_d_thc_calculated) return _d_adj_inv_gab_d_thc[index_];

        _d_adj_inv_gab_d_thc[index_].setZero();

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
                    _Nf(joint_i, 0) += _joints[joint_j].getI()(0, 0)*_gravitational_acceleration*_joints[_tip_index[chain]].get_dGsr_dTh(0, joint_i)(2, 3);
                }
            }
        }

        return _Nf;
    }

    // Angular Velocity to Angle (for Visualization)
    Eigen::Matrix<double, -1, 1> Manipulator::angularVelocity2Angle(const Eigen::Matrix<double, -1, 1> &angular_velocity_)
    {
        _angular_velocity = angular_velocity_;

        if(_is_first_time_measurement)
        {
            _is_first_time_measurement = false;
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

    // Subscriber
    void Manipulator::setECEnable(const bool &ec_enable_)
    {
        if(_ec_enable != ec_enable_)
        {
            _ec_enable = ec_enable_;
            for(int ik_index = 0; ik_index < _ik_index; ik_index++)
            {
                _interpolation[ik_index].setPoint(getPose(_end_joint_index[ik_index]), _target_pose[ik_index]);
            }
        }
    }

    void Manipulator::setEmergencyStop(const bool &emergency_stop_)
    {
        _emergency_stop = emergency_stop_;
    }

    void Manipulator::setMotorEnable(const bool &motor_enable_)
    {
        _motor_enable = motor_enable_;
    }

    void Manipulator::setTargetAngle(const Eigen::Matrix<double, -1, 1> &target_angle_)
    {
        int size_ = target_angle_.rows();
        if(size_ > _JOINT_NUM) size_ = _JOINT_NUM;

        for(int joint = 0; joint < size_; joint++)
        {
            _target_angle(joint, 0) = target_angle_(joint, 0);
        }
    }

    void Manipulator::setTargetPose(const Eigen::Matrix<double, -1, 1> &target_pose_)
    {
        // 8 = 2 + 6 = 1:start_joint + 1:end_joint + 3:position + 3:orientation
        int ik_index_num_ = target_pose_.rows()/8;
            _interpolation.resize(ik_index_num_);
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

    // Debug
    void Manipulator::printTree()
    {
        std::cout << _joints[0].getChildrenList() << std::endl;
    }

    void Manipulator::print()
    {
        // for(int i = 0; i < _ik_index; i++)
        // {
        //     std::cout << _start_joint_index[i] << " to " << _end_joint_index[i] << std::endl;
        //     std::cout << getPose(_end_joint_index[i]) << std::endl << std::endl;
        //     getPose(_end_joint_index[i]);
        // }
        // for(int i = 0; i < _JOINT_NUM; i++)
        // {
        //     std::cout << i << "\n";
        //     // std::cout << getJacobianBlock(0) << "\n";
        //     std::cout << getJacobianG(i) << "\n\n";
        // }
        std::cout << "Mf\n" << getMf() << std::endl << std::endl;
        std::cout << "Cf\n" << getCf() << std::endl << std::endl;
        std::cout << "Nf\n" << getNf() << std::endl << std::endl;

        // for(int i = 0; i < _JOINT_NUM; i++)
        // {
        //     for(int j = 0; j < _JOINT_NUM; j++)
        //     {
        //         for(int k = 0; k < _JOINT_NUM; k++)
        //         {
        //             std::cout << i << " " << j << " " << k << std::endl;
        //             // std::cout << get_dAdjointInverseGab_dThc(i, j, k) << std::endl << std::endl;
        //             // std::cout << EigenUtility.adjointInverse(_joints[i].getParentGsrTheta(j)) << std::endl << std::endl;
        //             // std::cout << get_dMab_dThc(i, j, k) << std::endl << std::endl;
        //             // std::cout << (_joints[k].getXi()).transpose() << std::endl << std::endl;
        //             // std::cout << getCfBlock(i, j) << std::endl << std::endl;
        //         }
        //     }
        // }
    }

    Eigen::Matrix<double, 6, 1> Manipulator::getTargetPose(const int &ik_index_)
    {
        return _target_pose[ik_index_];
    }

    Eigen::Matrix<double, 6, 1> Manipulator::getMidPose(const int &ik_index_)
    {
        if(_ec_enable) return _interpolation[ik_index_].getSigmoidInterpolation();

        return Eigen::Matrix<double, 6, 1>::Zero();
    }
}