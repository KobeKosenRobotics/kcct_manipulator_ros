#include "ec_calculator/model.h"

namespace ec_calculator
{
    Model::Model()
    {
        _chain_matrix.resize(_chain_num, _joint_num);
        _chain_matrix <<
        // 1  2  3  4  5  6  7  8  9 10
        1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
        1, 1, 0, 0, 1, 0, 0, 1, 0, 0;
    }

    void Model::changeModel(const int &joint_num_, const int &chain_num_, const Eigen::Matrix<bool, -1, -1> chain_mat_)
    {
        _joint_num = joint_num_;
        _chain_num = chain_num_;

        _chain_matrix.resize(_chain_num, _joint_num);
        _chain_matrix = chain_mat_;
    }

    int Model::getJointNum()
    {
        return _joint_num;
    }

    int Model::getChainNum()
    {
        return _chain_num;
    }

    Eigen::Matrix<bool, -1, -1> Model::getChainMat()
    {
        return _chain_matrix;
    }
}