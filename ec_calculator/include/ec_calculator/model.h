#ifndef MODEL_H

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
            // Eigen::Matrix<double, -1, -1> _joint_position, _translation_axis, _rotation_axis;

        public:
            Model();

            void changeModel(const int &chain_num_, const int &joint_num_, const Eigen::Matrix<bool, -1, -1> chain_mat_);    // TODO: Add _joint_position, etc. to the argument

            int getJointNum();
            int getChainNum();

            Eigen::Matrix<bool, -1, -1> getChainMat();
    };
}

#define MODEL_H
#endif