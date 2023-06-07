#ifndef EC_CALCULATOR_MODEL_H

#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ec_calculator
{
    class Model
    {
        private:
            bool _torque_control_enable;

            int _chain_num;
            int _joint_num;

            // Matrix
            Eigen::Matrix<bool, -1, -1> _chain_mat;
            Eigen::Matrix<double, 3, -1> _joint_position_link;
            Eigen::Matrix<double, 3, -1> _translation_axis;
            Eigen::Matrix<double, 3, -1> _rotation_axis;

            Eigen::Matrix<double, 10, -1> _inertia; // mass [kg], moment of inertia [kg*m^2]:Ixx,Ixy,Ixz,Iyx,Iyy,Iyz,Izx,Izy,Izz
            Eigen::Matrix<double, 3, -1> _center_of_gravity_link;

            // Gain
            Eigen::Matrix<double, -1, -1> _angle_2_angular_velocity_gain;
            double _ec_gain;

            // Gravitational Acceleration
            double _gravitational_acceleration; // [m/s^2]

        public:
            // Constructor
            Model();

            // Change Model
            // Velocity Control
            void changeModel(const int &chain_num_,
                            const int &joint_num_,
                            const Eigen::Matrix<bool, -1, -1> &chain_mat_,
                            const Eigen::Matrix<double, 3, -1> &joint_position_link_,
                            const Eigen::Matrix<double, 3, -1> &translation_axis_,
                            const Eigen::Matrix<double, 3, -1> &rotation_axis_,
                            const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_,
                            const double &ec_gain_);
            // Torque Control
            void changeModel(const int &chain_num_,
                            const int &joint_num_,
                            const Eigen::Matrix<bool, -1, -1> &chain_mat_,
                            const Eigen::Matrix<double, 3, -1> &joint_position_link_,
                            const Eigen::Matrix<double, 3, -1> &translation_axis_,
                            const Eigen::Matrix<double, 3, -1> &rotation_axis_,
                            const Eigen::Matrix<double, 10, -1> &inertia_,
                            const Eigen::Matrix<double, 3, -1> &center_of_gravity_link_,
                            const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_,
                            const double &ec_gain_);

                void changeTorqueControlEnable(const bool &torque_control_enable_);
                void changeChainNum(const int &chain_num_);
                void changeJointNum(const int &joint_num_);
                void changeChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_mat_);
                void changeJointPositionLink(const Eigen::Matrix<double, 3, -1> &joint_position_link_);
                void changeTranslationAxis(const Eigen::Matrix<double, 3, -1> &translation_axis_);
                void changeRotationAxis(const Eigen::Matrix<double, 3, -1> &rotation_axis_);

                void changeInertia(const Eigen::Matrix<double, 10, -1> &inertia_);
                void changeCenterOfGravityLink(const Eigen::Matrix<double, 3, -1> &center_of_gravity_link_);

                void changeAngle2AngularVelocityGain(const Eigen::Matrix<double, -1, -1> &angle_2_angular_velocity_gain_);
                void changeECGain(const double &ec_gain_);

                void changeGravitationalAcceleration(const double &gravitational_acceleration_);

            // Parameter Getters
            bool getTorqueControlEnable();
            int getJointNum();
            int getChainNum();
            Eigen::Matrix<bool, -1, -1> getChainMat();
            Eigen::Matrix<double, 3, 1> getJointPositionLink(const int &joint_);
            Eigen::Matrix<double, 3, 1> getTranslationAxis(const int &joint_);
            Eigen::Matrix<double, 3, 1> getRotationAxis(const int &joint_);

            Eigen::Matrix<double, 6, 6> getInertia(const int &joint_);
            Eigen::Matrix<double, 3, 1> getCenterOfGravityLink(const int &joint_);

            Eigen::Matrix<double, -1, -1> getAngle2AngularVelocityGain();
            double getECGain();

            double getGravitationalAcceleration();
    };
}

#define EC_CALCULATOR_MODEL_H
#endif