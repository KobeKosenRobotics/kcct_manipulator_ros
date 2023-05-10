#include "ec_calculator/model.h"

namespace ec_calculator
{
    // Constructor
    Model::Model()
    {
        _chain_mat.resize(_chain_num, _joint_num);
        _chain_mat <<
        // 1  2  3  4  5  6  7  8  9 10
        1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1,
        1, 1, 0, 0, 1, 0, 0, 1, 0, 0;

        _joint_position_link.resize(3, _joint_num);
        _joint_position_link <<
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 1, 2, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _tool_position_link.resize(3, _joint_num);
        _tool_position_link <<
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _translation_axis.resize(3, _joint_num);
        _translation_axis <<
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _rotation_axis.resize(3, _joint_num);
        _rotation_axis <<
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    }

    // Change Model
    void Model::changeModel(const int &chain_num_, const int &joint_num_, const Eigen::Matrix<bool, -1, -1> &chain_mat_, const Eigen::Matrix<double, 3, -1> &joint_position_link_, const Eigen::Matrix<double, 3, -1> &tool_position_link_, const Eigen::Matrix<double, 3, -1> &translation_axis_, const Eigen::Matrix<double, 3, -1> &rotation_axis_)
    {
        changeChainNum(chain_num_);
        changeJointNum(joint_num_);
        changeChainMatrix(chain_mat_);
        changeJointPositionLink(joint_position_link_);
        changeToolPositionLink(tool_position_link_);
        changeTranslationAxis(translation_axis_);
        changeRotationAxis(rotation_axis_);
    }

    void Model::changeChainNum(const int &chain_num_)
    {
        _chain_num = chain_num_;
    }

    void Model::changeJointNum(const int &joint_num_)
    {
        _joint_num = joint_num_;
    }

    void Model::changeChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_mat_)
    {
        if(chain_mat_.rows() == _chain_num && chain_mat_.cols() == _joint_num)
        {
            _chain_mat.resize(_chain_num, _joint_num);
            _chain_mat = chain_mat_;
        }
        else
        {
            std::cout << "Matrix(chain_mat_) Size do not match Chain Number or Joint Number." << std::endl;
        }
    }

    void Model::changeJointPositionLink(const Eigen::Matrix<double, 3, -1> &joint_position_link_)
    {
        if(joint_position_link_.cols() == _joint_num)
        {
            _joint_position_link.resize(3, _joint_num);
            _joint_position_link = joint_position_link_;
        }
        else
        {
            std::cout << "Matrix(joint_position_link_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeToolPositionLink(const Eigen::Matrix<double, 3, -1> &tool_position_link_)
    {
        if(tool_position_link_.cols() == _joint_num)
        {
            _tool_position_link.resize(3, _joint_num);
            _tool_position_link = tool_position_link_;
        }
        else
        {
            std::cout << "Matrix(tool_position_link_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeCenterOfGravityLink(const Eigen::Matrix<double, 3, -1> &center_of_gravity_link_)
    {
        if(center_of_gravity_link_.cols() == _joint_num)
        {
            _center_of_gravity_link.resize(3, _joint_num);
            _center_of_gravity_link = center_of_gravity_link_;
        }
        else
        {
            std::cout << "Matrix(center_of_gravity_link_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeTranslationAxis(const Eigen::Matrix<double, 3, -1> &translation_axis_)
    {
        if(translation_axis_.cols() == _joint_num)
        {
            _translation_axis.resize(3, _joint_num);
            _translation_axis = translation_axis_;
        }
        else
        {
            std::cout << "Matrix(translation_axis_) Size do not match Joint Number." << std::endl;
        }
    }

    void Model::changeRotationAxis(const Eigen::Matrix<double, 3, -1> &rotation_axis_)
    {
        if(rotation_axis_.cols() == _joint_num)
        {
            _rotation_axis.resize(3, _joint_num);
            _rotation_axis = rotation_axis_;
        }
        else
        {
            std::cout << "Matrix(rotation_axis_) Size do not match Joint Number." << std::endl;
        }
    }

    // Parameter Getters
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

    Eigen::Matrix<double, 3, 1> Model::getJointPositionLink(const int &joint_)
    {
        return _joint_position_link.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getToolPositionLink(const int &joint_)
    {
        return _tool_position_link.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getCenterOfGravityLink(const int &joint_)
    {
        return _center_of_gravity_link.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getTranslationAxis(const int &joint_)
    {
        return _translation_axis.col(joint_);
    }

    Eigen::Matrix<double, 3, 1> Model::getRotationAxis(const int &joint_)
    {
        return _rotation_axis.col(joint_);
    }
}