#include "ec_calculator/model.h"

namespace ec_calculator
{
    Model::Model()
    {
        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
        // 1  2  3  4  5  6  7  8  9 10
        1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
        1, 1, 0, 0, 1, 0, 0, 1, 0, 0;
    }

    void Model::changeModel(const int &chain_num_, const int &joint_num_, const Eigen::Matrix<bool, -1, -1> chain_mat_)
    {
        if(chain_num_ == chain_mat_.rows() && joint_num_ == chain_mat_.cols())
        {
            _chain_num = chain_num_;
            _joint_num = joint_num_;

            _chain_mat.resize(_chain_num, _joint_num);
            _chain_mat = chain_mat_;
        }
        else
        {
            std::cout << "Matrix Size do not match Chain Number or Joint Number." << std::endl;
        }
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
        return _chain_mat;
    }
}