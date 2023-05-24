#ifndef EC_CALCULATOR_MANIPULATOR_H

#include "model.h"
#include "joint.h"

#include <chrono>

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

            // Parameters
            Eigen::Matrix<double, -1, 1> _angle;
            Eigen::Matrix<double, -1, 1> _target_angle;
            Eigen::Matrix<double, -1, 1> _angular_velocity;
            Eigen::Matrix<double, -1, 1> _target_angular_velocity;
            int _start_joint = 0, _end_joint = 0;
            Eigen::Matrix<double, 6, 1> _target_pose;
            bool _ec_enable = false;
            bool _emergency_stop = false;
            bool _motor_enable = false;

            // Time
            std::chrono::system_clock::time_point _start_time, _end_time;
            double _during_time;
            bool _is_first_time_measurement = true; // TODO: Whether to reset when changing models

        public:
            // Initialize
            void init(Model* model_);
                void clearParameters();
                bool setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_matrix);
                void setJointParameters();
                    void setTipIndex(const int &tip_index_);
                    void clearTipIndex();
                void setAngle2AngularVelocityGain(const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_);
                void setECGain(const double &ec_gain_);

            // Properties
            int getChainNum();
            int getJointNum();
            std::string getJointName(const int &joint_index_);
            std::string getJointParentName(const int &joint_index_);

            // Visualize
            double getVisualData(const int &joint_index_, const int &data_index_);

            // Forward Kinematics
            void updateAngle(const Eigen::Matrix<double, -1, 1> &angle_);
            Eigen::Matrix<double, -1, 1> getAngle();
            Eigen::Matrix<double, 6, 1> getPose(const int &joint_index_);

            // Angle Command
            Eigen::Matrix<double, -1, 1> getAngularVelocityByAngle(const Eigen::Matrix<double, -1, 1> &target_angle_);
            Eigen::Matrix<double, -1, 1> getAngularVelocityByAngle();

            // Inverse Kinematics
            void setJointPosition(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> target_velocity_);
            void setJointVelocity(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> target_velocity_);
            Eigen::Matrix<double, -1, 1> getAngularVelocityByEC();

            // Angular Velocity to Angle (for Visualization)
            Eigen::Matrix<double, -1, 1> angularVelocity2Angle(const Eigen::Matrix<double, -1, 1> &angular_velocity_);

            // Subscriber
            void setECEnable(const bool &ec_enable_);
            void setEmergencyStop(const bool &emergency_stop_);
            void setMotorEnable(const bool &motor_enable_);
            void setTargetAngle(const Eigen::Matrix<double, -1, 1> &target_angle_);
            void setTargetPose(const Eigen::Matrix<double, -1, 1> &target_pose_);    // 2: start_joint, end_joint, 6: 3position, 3orientation

            // Debug
            void printTree();
            void print();
    };
}

#define EC_CALCULATOR_MANIPULATOR_H
#endif