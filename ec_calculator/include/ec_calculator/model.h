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
            Eigen::Matrix<double, 3, -1> _link;
            Eigen::Matrix<double, 3, -1> _translation_axis;
            Eigen::Matrix<double, 3, -1> _rotation_axis;

        public:
            Model();

            void changeModel(const int &chain_num_, const int &joint_num_, const Eigen::Matrix<bool, -1, -1> &chain_mat_, const Eigen::Matrix<double, 3, -1> &link_, const Eigen::Matrix<double, 3, -1> &translation_axis_, const Eigen::Matrix<double, 3, -1> &rotation_axis_);    // TODO: Add _joint_position, etc. to the argument

            int getJointNum();
            int getChainNum();

            Eigen::Matrix<bool, -1, -1> getChainMat();
            Eigen::Matrix<double, 3, 1> getLink(const int &joint_);
            Eigen::Matrix<double, 3, 1> getTranslationAxis(const int &joint_);
            Eigen::Matrix<double, 3, 1> getRotationAxis(const int &joint_);
    };
}

#define EC_CALCULATOR_MODEL_H
#endif