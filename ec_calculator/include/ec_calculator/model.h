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
            int _joint_num = 10;
            int _chain_num = 3;
            Eigen::Matrix<bool, -1, -1> _chain_matrix;
            // Eigen::Matrix<double, -1, -1> _joint_position, _translation_axis, _rotation_axis;

        public:
            Model();
            // const int JOINT_NUM = 10;
            // const int CHAIN_NUM = 3;
            int getJointNum();
            int getChainNum();
            Eigen::Matrix<bool, -1, -1> getChainMat();
    };
}

#define MODEL_H
#endif