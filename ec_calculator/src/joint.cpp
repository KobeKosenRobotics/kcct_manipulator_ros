#include "ec_calculator/joint.h"
#include "ec_calculator/eigenUtility.h"

namespace ec_calculator
{
    Joint::Joint()
    {
        updateTheta(_theta);
    }

    // Family
    void Joint::setJoint(const int  &joint_index_)
    {
        _joint_index = joint_index_;
    }

    void Joint::setJointProperty()
    {
        _q = getQ();
        // _v = tree_property.getV(_joint_index);
        // _w = tree_property.getW(_joint_index);

        if(_v.norm() == 0)
        {
            _v = -_w.cross(_q);
        }

        getXi();

        getGsjZero();
    }

    Eigen::Matrix<double, 3, 1> Joint::getQ()
    {
        // if(_parent_joint == nullptr)
        // {
        //     return tree_property.getLink(_joint_index);
        // }
        // return _parent_joint->_q+tree_property.getLink(_joint_index);

        // if Property::_joint_position is defined in world coordinate
        // return tree_property.getQ(_joint);
        return _q;
    }

    Eigen::Matrix<double, 6, 1> Joint::getXi()
    {
        _xi << -_w.cross(_q), _w;

        return _xi;
    }

    Eigen::Matrix<double, 4, 4> Joint::getGsjZero()
    {
        _gsj_zero <<
        EigenUtility.getIdentity3(), _q,
        0.0, 0.0, 0.0, 1.0;

        return _gsj_zero;
    }

    void Joint::setParent(Joint *parent_joint_)
    {
        _parent_joint = parent_joint_;
    }

    void Joint::setChildren(Joint *children_joint_)
    {
        _children_joint[_children_number] = children_joint_;
        _children_number++;
    }

    void Joint::printJoint()
    {
        std::cout << "joint: " << _joint_index << std::endl;
        std::cout << "parent: " << _parent_joint->_joint_index << std::endl;
        for(int i = 0; i < _children_number; i++)
        {
            if(i == 0)
            {
                std::cout << "children: ";
            }
            std::cout << _children_joint[i]->_joint_index << " ";
            if(i == _children_number-1)
            {
                std::cout << std::endl;
            }
        }
    }

    // Parameter
    void Joint::updateTheta(const double &theta_)
    {
        _theta = theta_;
        _cos_theta = cos(theta_);
        _sin_theta = sin(theta_);
        _v_theta = 1 - _cos_theta;
    }

    // Parameter
    Eigen::Matrix<double, 4, 4> Joint::getExpXiHatTheta()
    {
        updateTheta(_theta);
        getExpWHatTheta();

        // _exp_xi_hat_theta <<
        // _exp_w_hat_theta, ((tree_base.getIdentity3() - _exp_w_hat_theta) * (_w.cross(_v))) + (_w*_w.transpose()) * _v*_theta,
        // 0.0, 0.0, 0.0, 1.0;

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

    Eigen::Matrix<double, 4, 4> Joint::getGsjTheta()
    {
        return getGsjThetaRecursion()*_gsj_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::getGsjThetaRecursion()
    {
        if(_parent_joint == nullptr)
        {
            std::cout << std::endl;
            return getExpXiHatTheta();
        }
        if(_joint_index >= JOINT_NUMBER)
        {
            return _parent_joint->getGsjThetaRecursion();
        }

        return _parent_joint->getGsjThetaRecursion()*getExpXiHatTheta();
    }

    Eigen::Matrix<double, 4, 4> Joint::getChildrenExpXiHatTheta(const int  &minimum_joint_)
    {
        if(_joint_index < JOINT_NUMBER) std::cout << "ERROR: _joint < JOINT_NUMBER" << std::endl;

        _minimum_joint = minimum_joint_;
        _parent_joint->_minimum_joint = _minimum_joint;

        return getChildrenExpXiHatThetaRecursion()*_gsj_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::getChildrenExpXiHatThetaRecursion()
    {
        if(_joint_index == _minimum_joint)
        {
            return getExpXiHatTheta();
        }

        _parent_joint->_minimum_joint = _minimum_joint;

        if(_joint_index >= JOINT_NUMBER)
        {
            return _parent_joint->getChildrenExpXiHatThetaRecursion();
        }

        return _parent_joint->getChildrenExpXiHatThetaRecursion()*getExpXiHatTheta();
    }
}