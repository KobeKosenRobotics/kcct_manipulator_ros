#include "ec_calculator/manipulator.h"

namespace ec_calculator
{
    void Manipulator::init()
    {
        for(int index = 0; index < JOINT_NUM; index++)
        {
            _joints[index].init(index, "Joint" + std::to_string(index));
        }
    }

    bool Manipulator::setChainMatrix(Eigen::Matrix<bool, CHAIN_NUM, JOINT_NUM> chain_mat)
    {
        std::cout << "\nSetChainMatrix:" << std::endl;
        for(int chain = 0; chain < CHAIN_NUM; chain++)
        {
            int parent = -1;
            for(int child = 0; child < JOINT_NUM; child++)
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
    }

    Joint Manipulator::getJoint(int index)
    {
        return _joints[index];
    }

    void Manipulator::printTree()
    {
        std::cout << _joints[0].getChildrenList() << std::endl;
    }
}