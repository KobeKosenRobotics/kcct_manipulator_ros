#ifndef EC_CALCULATOR_MANIPULATOR_H

#include "model.h"
#include "joint.h"

namespace ec_calculator
{
    class Manipulator
    {
        private:
            Model* _model;
            std::vector<Joint> _joints;
            int _JOINT_NUM, _CHAIN_NUM;

        public:
            void init(Model* model_);
            bool setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_matrix);

            Joint getJoint(int index);

            void printTree();
    };
}

#define EC_CALCULATOR_MANIPULATOR_H
#endif