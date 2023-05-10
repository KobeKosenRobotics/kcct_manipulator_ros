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
            int _chain_num = 3;
            int _joint_num = 10;

            Eigen::Matrix<bool, -1, -1> _chain_mat;
            Eigen::Matrix<double, 3, -1> _joint_position_link;
            Eigen::Matrix<double, 3, -1> _tool_position_link;
            Eigen::Matrix<double, 3, -1> _center_of_gravity_link;
            Eigen::Matrix<double, 3, -1> _translation_axis;
            Eigen::Matrix<double, 3, -1> _rotation_axis;

        public:
            // Constructor
            Model();

            // Change Model
            void changeModel(const int &chain_num_, const int &joint_num_, const Eigen::Matrix<bool, -1, -1> &chain_mat_, const Eigen::Matrix<double, 3, -1> &joint_position_link_, const Eigen::Matrix<double, 3, -1> &tool_position_link_, const Eigen::Matrix<double, 3, -1> &translation_axis_, const Eigen::Matrix<double, 3, -1> &rotation_axis_);
                void changeChainNum(const int &chain_num_);
                void changeJointNum(const int &joint_num_);
                void changeChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_mat_);
                void changeJointPositionLink(const Eigen::Matrix<double, 3, -1> &joint_position_link_);
                void changeToolPositionLink(const Eigen::Matrix<double, 3, -1> &tool_position_link_);
                void changeCenterOfGravityLink(const Eigen::Matrix<double, 3, -1> &center_of_gravity_link_);
                void changeTranslationAxis(const Eigen::Matrix<double, 3, -1> &translation_axis_);
                void changeRotationAxis(const Eigen::Matrix<double, 3, -1> &rotation_axis_);

            // Parameter Getters
            int getJointNum();
            int getChainNum();
            Eigen::Matrix<bool, -1, -1> getChainMat();
            Eigen::Matrix<double, 3, 1> getJointPositionLink(const int &joint_);
            Eigen::Matrix<double, 3, 1> getToolPositionLink(const int &joint_);
            Eigen::Matrix<double, 3, 1> getCenterOfGravityLink(const int &joint_);
            Eigen::Matrix<double, 3, 1> getTranslationAxis(const int &joint_);
            Eigen::Matrix<double, 3, 1> getRotationAxis(const int &joint_);
    };
}

#define EC_CALCULATOR_MODEL_H
#endif