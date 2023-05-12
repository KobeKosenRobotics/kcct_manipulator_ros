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
            Eigen::Matrix<double, -1, -1> _angle_2_angular_velocity_gain;
            double _ec_gain;

        public:
            // Initialize
            void init(Model* model_);
                bool setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_matrix);
                void setJointParameters();
                    void setTipIndex(const int &tip_index_);
                    void clearTipIndex();
                void setAngle2AngularVelocityGain(const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_);
                void setECGain(const double &ec_gain_);

            // Forward Kinematics
            Eigen::Matrix<double, 6, 1> getPose(const int &joint_index_);

            // Angle to Angular Velocity
            Eigen::Matrix<double, -1, 1> getAngularVelocityByAngle(const Eigen::Matrix<double, -1, 1> &angle_);

            // Inverse Kinematics
            Eigen::Matrix<double, -1, 1> getAngularVelocityByEC(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> target_velocity_);

            // Debug
            void printTree();
            void print();
    };
}

#define EC_CALCULATOR_MANIPULATOR_H
#endif