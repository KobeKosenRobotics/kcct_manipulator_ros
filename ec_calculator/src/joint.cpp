#include "ec_calculator/joint.h"
#include "ec_calculator/eigenUtility.h"

namespace ec_calculator
{
    // Constructor
    Joint::Joint()
    {
        updateTheta(0.0);
    }

    // Initialize
    void Joint::init(const int index, const std::string name)
    {
        updateTheta(0.0);
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

    std::string Joint::getParentName()
    {
        if(_parent == nullptr) return "manipulator_base_link";
        return _parent->getName();
    }

    int Joint::getNumOfParentGenerations()
    {
        if(_index <= 0) return 0;
        return _parent->getNumOfParentGenerations()+1;
    }

    bool Joint::isTipJoint()
    {
        if(_children.size() == 0) return true;
        return false;
    }

    // Parameters Setters
    void Joint::setParameters(Model *model_)
    {
        clearParameters();
        setQ(model_->getJointPositionLink(_index));
        setW(model_->getRotationAxis(_index));
        setV(model_->getTranslationAxis(_index));   // "setV()" must be executed after "setW()"
        setXi();
        setGstZero();
    }

    void Joint::clearParameters()
    {
        _q.setZero();
        _v.setZero();
        _w.setZero();
        _lp.setZero();
        _xi.setZero();
        _gst_zero.setZero();
    }

    void Joint::setQ(const Eigen::Matrix<double, 3, 1> &joint_position_link_)
    {
        _lp = joint_position_link_;

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
        _xi << _v, _w;
    }

    void Joint::setGstZero()
    {
        _gst_zero <<
        EigenUtility.getIdentity3(), _q,
        0.0, 0.0, 0.0, 1.0;
    }

    // Visualize
    double Joint::getVisualData(const int &index_)
    {
        setVisualData();
        return _visual_data(index_, 0);
    }

    void Joint::setVisualData()
    {
        _visual_data.block(3, 0, 3, 1) = _theta * _w;

        if(_parent == nullptr)
        {
            _visual_data.block(0, 0, 3, 1) = _lp;
            return;
        }

        if(_parent->_w.norm() == 0)
        {
            _visual_data.block(0, 0, 3, 1) = _parent->_theta *_parent-> _v;
            return;
        }

        _visual_data.block(0, 0, 3, 1) = _lp;
        return;
    }

    Eigen::Matrix<double, 6, 1> Joint::getVisualData()
    {
        setVisualData();
        return _visual_data;
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
        if(_w.norm() == 0)
        {
            _exp_xi_hat_theta <<
            EigenUtility.getIdentity3(), _theta*_v,
            0.0, 0.0, 0.0, 1.0;

            return _exp_xi_hat_theta;
        }

        getExpWHatTheta();

        _exp_xi_hat_theta <<
        _exp_w_hat_theta, ((EigenUtility.getIdentity3() - _exp_w_hat_theta) * (_w.cross(_v))) + (_w*_w.transpose()) * _v * _theta,
        0.0, 0.0, 0.0, 1.0;

        return _exp_xi_hat_theta;
    }

    Eigen::Matrix<double, 3, 3> Joint::getExpWHatTheta()
    {
        if(_w.norm() == 0) return EigenUtility.getIdentity3();

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
        if(_parent == nullptr) return _gst_zero;
        return _parent->getGstThetaRecursion()*_gst_zero;
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
        if(_parent == nullptr) return _gst_zero;

        _minimum_index = minimum_index_;
        _parent->_minimum_index = _minimum_index;

        return _parent->getParentGstThetaRecursion()*_gst_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::getParentGstThetaRecursion()
    {
        if(_index == _minimum_index) return getExpXiHatTheta();

        _parent->_minimum_index = _minimum_index;

        return _parent->getParentGstThetaRecursion()*getExpXiHatTheta();
    }

    // Debug
    std::string Joint::getChildrenList()
    {
        std::cout << "\nTree:" << std::endl;
        return getChildrenList(getNumOfParentGenerations());
    }

    std::string Joint::getChildrenList(const int tab)
    {
        std::string tree;
        if(_parent == nullptr) tree += " --->\t";
        tree += std::to_string(_index);
        tree += " --->\t";

        if(_children.size() == 0) return tree + "\n\n";

        tree += _children[0]->getChildrenList(getNumOfParentGenerations()+1);

        for(int child = 1; child < _children.size(); child++)
        {
            for(int t = 0; t <= tab; t++)
            {
                tree += "\t";
            }
            tree += "'---->\t";
            tree += _children[child]->getChildrenList(getNumOfParentGenerations()+1);
        }

        return tree;
    }
}