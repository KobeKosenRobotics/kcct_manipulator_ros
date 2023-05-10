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

        for(int index = 0; index < _JOINT_NUM; index++)
        {
            _joints[index].init(index, "Joint" + std::to_string(index));
        }

        setChainMatrix(_model->getChainMat());

        setJointParameters();
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
        for(int joint = 0; joint < _JOINT_NUM; joint++)
        {
            _joints[joint].setParameters(_model);
        }
    }

    // Joint Manipulator::getJoint(int index)
    // {
    //     return _joints[index];
    // }

    // Forward Kinematics
    Eigen::Matrix<double, 6, 1> Manipulator::getPose(const int &joint_index_)
    {
        return EigenUtility.getPose(_joints[joint_index_].getGstTheta());
    }

    // Debug
    void Manipulator::printTree()
    {
        std::cout << _joints[0].getChildrenList() << std::endl;
    }

    void Manipulator::print()
    {
        for(int joint_index = 0; joint_index < _JOINT_NUM; joint_index++)
        {
            std::cout << "joint" << joint_index << ":\t" << getPose(joint_index).transpose() << std::endl << std::endl;
        }
    }
}