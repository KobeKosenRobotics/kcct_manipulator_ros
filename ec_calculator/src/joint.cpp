#include "ec_calculator/joint.h"
#include "ec_calculator/eigenUtility.h"

namespace ec_calculator
{
    // Constructor
    Joint::Joint()
    {
        updateTheta(_theta);
    }

    // Initialize
    void Joint::init(const int index, const std::string name)
    {
        _index = index;
        _name = name;
        clearChildren();
    }

    // Chain Management Functions
    void Joint::setParent(Joint &parent)
    {
        _parent = &parent;
    }

    bool Joint::addChild(Joint &child)
    {
        if(std::find(_children.begin(), _children.end(), &child) != _children.end()) return false;
        _children.push_back(&child);
        return true;
    }

    void Joint::clearChildren()
    {
        _children.clear();
    }

    // Properties
    int Joint::getIndex()
    {
        return _index;
    }

    std::string Joint::getName()
    {
        return _name;
    }

    std::string Joint::getChildrenList()
    {
        std::cout << "\nTree:" << std::endl;
        return getChildrenList(0);
    }

    int Joint::getNumOfParentGenerations()
    {
        if(_index <= 0) return 0;
        return _parent->getNumOfParentGenerations()+1;
    }

    // Parameters Setters
    void Joint::setParameters(Model *model_)
    {
        setQ(model_->getJointPositionLink(_index));
        setW(model_->getRotationAxis(_index));
        setV(model_->getTranslationAxis(_index));   // "setV()" must be executed after "setW()"
        setGstZero(model_->getToolPositionLink(_index));
    }

    void Joint::setQ(const Eigen::Matrix<double, 3, 1> &joint_position_link_)
    {
        if(_parent == nullptr)
        {
            _q = joint_position_link_;
        }
        else
        {
            _q = _parent->_q + joint_position_link_;
        }
    }

    void Joint::setV(const Eigen::Matrix<double, 3, 1> &translation_axis_)
    {
        if(translation_axis_.norm() == 0)
        {
            _v = -_w.cross(_q);
        }
        else
        {
            _v = translation_axis_;
        }
    }

    void Joint::setW(const Eigen::Matrix<double, 3, 1> &rotation_axis_)
    {
        _w = rotation_axis_;
    }

    void Joint::setXi()
    {
        _xi << -_w.cross(_q), _w;
    }

    void Joint::setGstZero(const Eigen::Matrix<double, 3, 1> &tool_position_link_)
    {
        _gst_zero <<
        EigenUtility.getIdentity3(), _q + tool_position_link_,
        0.0, 0.0, 0.0, 1.0;
    }

    // Forward Kinematics
    void Joint::updateTheta(const double &theta_)
    {
        _theta = theta_;
        _cos_theta = cos(theta_);
        _sin_theta = sin(theta_);
        _v_theta = 1 - _cos_theta;
    }

    Eigen::Matrix<double, 4, 4> Joint::getExpXiHatTheta()
    {
        getExpWHatTheta();

        _exp_xi_hat_theta <<
        _exp_w_hat_theta, ((EigenUtility.getIdentity3() - _exp_w_hat_theta) * _w.cross(_v)) + _w*_w.transpose() * _v * _theta;
        0.0, 0.0, 0.0, 1.0;

        return _exp_xi_hat_theta;
    }

    Eigen::Matrix<double, 3, 3> Joint::getExpWHatTheta()
    {
        _exp_w_hat_theta(0,0) = pow(_w(0,0),2)*_v_theta + _cos_theta;
        _exp_w_hat_theta(0,1) = _w(0,0)*_w(1,0)*_v_theta - _w(2,0)*_sin_theta;
        _exp_w_hat_theta(0,2) = _w(0,0)*_w(2,0)*_v_theta + _w(1,0)*_sin_theta;

        _exp_w_hat_theta(1,0) = _w(0,0)*_w(1,0)*_v_theta + _w(2,0)*_sin_theta;
        _exp_w_hat_theta(1,1) = pow(_w(1,0),2)*_v_theta + _cos_theta;
        _exp_w_hat_theta(1,2) = _w(1,0)*_w(2,0)*_v_theta - _w(0,0)*_sin_theta;

        _exp_w_hat_theta(2,0) = _w(0,0)*_w(2,0)*_v_theta - _w(1,0)*_sin_theta;
        _exp_w_hat_theta(2,1) = _w(1,0)*_w(2,0)*_v_theta + _w(0,0)*_sin_theta;
        _exp_w_hat_theta(2,2) = pow(_w(2,0),2)*_v_theta + _cos_theta;

        return _exp_w_hat_theta;
    }

    Eigen::Matrix<double, 4, 4> Joint::getGstTheta()
    {
        return getGstThetaRecursion()*_gst_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::getGstThetaRecursion()
    {
        if(_parent == nullptr)
        {
            return getExpXiHatTheta();
        }

        return _parent->getGstThetaRecursion()*getExpXiHatTheta();
    }

    // Inverse Kinematics
    Eigen::Matrix<double, 6, 1> Joint::getXiDagger(const int &minimum_index_)
    {
        if(!isParent(minimum_index_)) return Eigen::Matrix<double, 6, 1>::Zero();

        return EigenUtility.adjointInverse(getParentGstTheta(minimum_index_))*getXi(minimum_index_);
    }

    bool Joint::isParent(const int &parent_index_)
    {
        if(_index == parent_index_) return true;
        if(_parent == nullptr) return false;
        return _parent->isParent(parent_index_);
    }

    Eigen::Matrix<double, 6, 1> Joint::getXi(const int &parent_index_)
    {
        if(_index == parent_index_) return _xi;
        return _parent->getXi(parent_index_);
    }

    Eigen::Matrix<double, 4, 4> Joint::getParentGstTheta(const int &minimum_index_)
    {
        _minimum_index = minimum_index_;
        _parent->_minimum_index = _minimum_index;

        return getParentGstThetaRecursion()*_gst_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::getParentGstThetaRecursion()
    {
        if(_index == _minimum_index) return getExpXiHatTheta();

        _parent->_minimum_index = _minimum_index;

        return _parent->getParentGstThetaRecursion()*getExpXiHatTheta();
    }

    // Debug
    std::string Joint::getChildrenList(const int tab)
    {
        std::string tree = std::to_string(_index);
        if(_children.size() == 0) return tree + "\n\n";
        tree += " --->\t";
        tree += _children[0]->getChildrenList(getNumOfParentGenerations());
        for(int child = 1; child < _children.size(); child++)
        {
            for(int t = 0; t <= tab; t++)
            {
                tree += "\t";
            }
            tree += "'---->\t";
            tree += _children[child]->getChildrenList(tab);
        }

        return tree;
    }
}