#ifndef EC_CALCULATOR_MANIPULATOR_H

#include "model.h"
#include "joint.h"

namespace ec_calculator
{
    class Manipulator
    {
        private:
            // Model
            Model* _model;
            int _JOINT_NUM, _CHAIN_NUM;
            std::vector<Joint> _joints;
            std::vector<int> _tip_index;   // joints[_tip_index[]] have no children.

        public:
            // Initialize
            void init(Model* model_);
            bool setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_matrix);
            void setJointParameters();
                void setTipIndex(const int &tip_index_);
                void clearTipIndex();

            // Forward Kinematics
            Eigen::Matrix<double, 6, 1> getPose(const int &joint_index_);

            // Inverse Kinematics
            Eigen::Matrix<double, -1, 1> getAngularVelocity(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> target_velocity_);

            // Debug
            void printTree();
            void print();
    };
}

#define EC_CALCULATOR_MANIPULATOR_H
#endif