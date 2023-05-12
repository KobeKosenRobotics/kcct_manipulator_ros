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
        _joints.resize(_JOINT_NUM);
        _angle.resize(_JOINT_NUM, 1);
        _target_angular_velocity.resize(_JOINT_NUM, 1);

        for(int index = 0; index < _JOINT_NUM; index++)
        {
            _joints[index].init(index, "Joint" + std::to_string(index));
        }

        setChainMatrix(_model->getChainMat());

        setJointParameters();

        setAngle2AngularVelocityGain(_model->getAngle2AngularVelocityGain());

        setECGain(_model->getECGain());
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
        return true;
    }

    void Manipulator::setJointParameters()
    {
        clearTipIndex();

        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            _joints[joint].setParameters(_model);

            if(_joints[joint].isTipJoint()) setTipIndex(joint);
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

    // Forward Kinematics
    Eigen::Matrix<double, 6, 1> Manipulator::getPose(const int &joint_index_)
    {
        return EigenUtility.getPose(_joints[joint_index_].getGstTheta());
    }

    // Angle to Angular Velocity
    Eigen::Matrix<double, -1, -1> Manipulator::getAngularVelocityByAngle(const Eigen::Matrix<double, -1, 1> &target_angle_)
    {
        _target_angular_velocity = _angle_2_angular_velocity_gain * (target_angle_ - _angle);
        return _target_angular_velocity;
    }

    // Debug
    void Manipulator::printTree()
    {
        std::cout << _joints[0].getChildrenList() << std::endl;
    }

    void Manipulator::print()
    {
        for(int i = 0; i < _tip_index.size(); i++)
        {
            std::cout << "joint" << _tip_index[i] << ":\t" << getPose(_tip_index[i]).transpose() << std::endl;
        }
    }
}