#ifndef EC_CALCULATOR_MANIPULATOR_H

#include "joint.h"

#ifndef JOINT_NUM
#define JOINT_NUM 10
#endif

#ifndef CHAIN_NUM
#define CHAIN_NUM 3
#endif

namespace ec_calculator
{
    class Manipulator
    {
        private:
            Joint _joints[JOINT_NUM];

        public:
            void init();
            bool setChainMatrix(Eigen::Matrix<bool, CHAIN_NUM, JOINT_NUM> chain_mat);
            
            Joint getJoint(int index);

            void printTree();
    };
}

#define EC_CALCULATOR_MANIPULATOR_H
#endif