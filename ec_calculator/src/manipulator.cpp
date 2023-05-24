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

    // // Inverse Kinematics
    // void Manipulator::setJointPose(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> &target_pose_)
    // {
    //     _start_joint_index[_ik_index] = start_joint_index_;
    //     _end_joint_index[_ik_index] = end_joint_index_;
    //     _target_pose[_ik_index] = target_pose_;
    //     _ik_index++;
    // }

    void Manipulator::setJointVelocity(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> &target_velocity_)
    {
        // _start_joint_index[_ik_index] = start_joint_index_;
        // _end_joint_index[_ik_index] = end_joint_index_;
        // _target_velocity[_ik_index] = target_velocity_;
        // _start_joint_index.push_back(start_joint_index_);
        // _end_joint_index.push_back(end_joint_index_);
        // _target_velocity.resize(1);
        // _target_velocity[0] = target_velocity_;
        _start_joint_index = start_joint_index_;
        _end_joint_index = end_joint_index_;
        _target_velocity = target_velocity_;
        _ik_index++;
    }

    void Manipulator::clearJointVelocity()
    {
        // _start_joint_index.clear();
        // _end_joint_index.clear();
        // _target_velocity.clear();
        _ik_index = 0;
    }

    Eigen::Matrix<double, -1, 1> Manipulator::getAngularVelocityByEC()
    {
        _target_angular_velocity = _ec_gain * EigenUtility.getPseudoInverseMatrix(getJacobian()) * (_target_velocity-getPose(_end_joint_index));

        // std::cout << _joints[_end_joint_index].getXiDagger(_start_joint_index) << std::endl;
        // std::cout << getJacobianBlock(0) << std::endl;
        // std::cout << getJacobian() << std::endl;
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
        // _jacobian_block[ik_index_].resize(6, _JOINT_NUM);
        _jacobian_block.resize(6, _JOINT_NUM);
        // _jacobian_block[ik_index_].setZero();
        _jacobian_block.setZero();

        // for(int ik_index = _start_joint_index[ik_index_]; ik_index < _end_joint_index[ik_index_]; ik_index++)
        // {
        //     _jacobian_block[ik_index_].block(0, ik_index, 6, 1) = _joints[_end_joint_index[ik_index_]].getXiDagger(ik_index);
        // }

        // _jacobian_block[ik_index_] = EigenUtility.getTransformationMatrix(_joints[_end_joint_index[ik_index_]].getGstTheta()) * _jacobian_block[ik_index_];

        // return _jacobian_block[ik_index_];
        for(int ik_index = _start_joint_index; ik_index < _end_joint_index; ik_index++)
        {
            // _jacobian_block.block(0, ik_index, 6, 1) = _joints[_end_joint_index].getXiDagger(ik_index);
            _jacobian_block(0, ik_index) = 10*ik_index + _end_joint_index;
        }

        // _jacobian_block = EigenUtility.getTransformationMatrix(_joints[_end_joint_index].getGstTheta()) * _jacobian_block;

        return _jacobian_block;
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
        _ec_enable = ec_enable_;
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
        // // 8 = 2 + 6 = 1:start_joint + 1:end_joint + 3:position + 3:orientation
        // int size_ = target_pose_.rows() - 2;
        // if(size_ < 0) return;
        // if(size_ > 6) size_ = 6;

        // _start_joint_index = (int)(target_pose_(0, 0) + 0.5);
        // _end_joint_index = (int)(target_pose_(1, 0) + 0.5);

        // for(int joint = 0; joint < size_; joint++)
        // {
        //     _target_pose(joint, 0) = target_pose_(joint+2, 0);
        // }
        // for(int joint = size_; joint < 6; joint++)
        // {
        //     _target_angle(joint, 0) = 0.0;
        // }
    }

    void Manipulator::setTargetVelocity(const Eigen::Matrix<double, -1, 1> &target_velocity_)
    {
        if(target_velocity_.rows() == 8)
        {
            setJointVelocity((int)(target_velocity_(0, 0)+0.5), (int)(target_velocity_(1, 0)+0.5), target_velocity_.block(2, 0, 6, 1));
        }
    }

    // Debug
    void Manipulator::printTree()
    {
        std::cout << _joints[0].getChildrenList() << std::endl;
    }

    void Manipulator::print()
    {
        // for(int i = 0; i < _tip_index.size(); i++)
        // {
        //     std::cout << "joint" << _tip_index[i] << ":   " << getPose(_tip_index[i]).transpose() << std::endl;
        // }
        // std::cout << std::endl;

        // std::cout << _ec_enable << "  " << _emergency_stop << "  " << _motor_enable << std::endl
        // << _target_angle.transpose() << std::endl
        // << _start_joint_index << "  " << _end_joint_index << std::endl
        // <<  _target_pose.transpose() << std::endl << std::endl;

        std::cout << getJacobianBlock(_ik_index) << std::endl << std::endl;
    }
}