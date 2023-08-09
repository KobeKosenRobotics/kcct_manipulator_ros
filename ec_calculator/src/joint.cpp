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

    bool Joint::isParent(const int &parent_index_)
    {
        if(_index == parent_index_) return true;
        if(_parent == nullptr) return false;
        return _parent->isParent(parent_index_);
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

    Eigen::Matrix<double, 6, 1> Joint::getXi()
    {
        return _xi;
    }

    Eigen::Matrix<double, 6, 1> Joint::getXi(const int &parent_index_)
    {
        if(_index == parent_index_) return _xi;
        return _parent->getXi(parent_index_);
    }

    Eigen::Matrix<double, 6, 6> Joint::getI()
    {
        return _i;
    }

    // Parameters Setters
    void Joint::setParameters(Model *model_)
    {
        clearParameters();
        setTorqueControlEnable(model_->getTorqueControlEnable());
        setQ(model_->getJointPositionLink(_index));
        setW(model_->getRotationAxis(_index));
        setV(model_->getTranslationAxis(_index));   // "setV()" must be executed after "setW()"
        setXi();
        setGstZero();

        if(_torque_control_enable)
        {
            setQg(model_->getCenterOfGravityLink(_index));
            setGsrZero();
            setInertia(model_->getInertia(_index));
        }
    }

    void Joint::clearParameters()
    {
        _q.setZero();
        _qg.setZero();
        _v.setZero();
        _w.setZero();
        _l.setZero();
        _lg.setZero();
        _xi.setZero();
        _xi_hat.setZero();
        _gst_zero.setZero();
        _gsr_zero.setZero();
        _i.setZero();
    }

    void Joint::setTorqueControlEnable(const bool &torque_control_enable_)
    {
        _torque_control_enable = torque_control_enable_;
    }

    void Joint::setQ(const Eigen::Matrix<double, 3, 1> &joint_position_link_)
    {
        _l = joint_position_link_;

        if(_parent == nullptr)
        {
            _q = _l;
        }
        else
        {
            _q = _parent->_q + _l;
        }
    }

    void Joint::setQg(const Eigen::Matrix<double, 3, 1> &center_of_gravity_link_)
    {
        _lg = center_of_gravity_link_;

        _qg = _q + _lg;
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

        _xi_hat <<
            EigenUtility.hat(_w), _v,
            0.0, 0.0, 0.0, 0.0;
    }

    void Joint::setGstZero()
    {
        _gst_zero <<
            EigenUtility.getIdentity3(), _q,
            0.0, 0.0, 0.0, 1.0;
    }

    void Joint::setGsrZero()
    {
        _gsr_zero <<
            EigenUtility.getIdentity3(), _qg,
            0.0, 0.0, 0.0, 1.0;
    }

    void Joint::setInertia(const Eigen::Matrix<double, 6, 6> &inertia_)
    {
        _i = inertia_;
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
            _visual_data.block(0, 0, 3, 1) = _l;
            return;
        }

        if(_parent->_w.norm() == 0)
        {
            _visual_data.block(0, 0, 3, 1) = _parent->_theta *_parent-> _v;
            return;
        }

        _visual_data.block(0, 0, 3, 1) = _l;
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
        _was_calculated = false;

        _theta = theta_;
        _cos_theta = cos(theta_);
        _sin_theta = sin(theta_);
        _v_theta = 1 - _cos_theta;

        getExpXiHatTheta();

        _was_calculated = true;
    }

    Eigen::Matrix<double, 4, 4> Joint::getExpXiHatTheta()
    {
        if(_was_calculated) return _exp_xi_hat_theta;

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

    Eigen::Matrix<double, 4, 4> Joint::getGsrTheta()
    {
        return getGstThetaRecursion()*_gsr_zero;
    }

    // Inverse Kinematics
    Eigen::Matrix<double, 6, 1> Joint::getXiDagger(const int &minimum_index_)
    {
        if(!isParent(minimum_index_)) return Eigen::Matrix<double, 6, 1>::Zero();

        return EigenUtility.adjointInverse(getParentGstTheta(minimum_index_))*getXi(minimum_index_);
    }

    Eigen::Matrix<double, 4, 4> Joint::getParentGstTheta(const int &minimum_index_)
    {
        if(!isParent(minimum_index_)) return Eigen::Matrix<double, 4, 4>::Zero();

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

    // Torque Control
    Eigen::Matrix<double, 6, 1> Joint::getXiDaggerG(const int &minimum_index_)
    {
        if(!isParent(minimum_index_)) return Eigen::Matrix<double, 6, 1>::Zero();

        return EigenUtility.adjointInverse(getParentGsrTheta(minimum_index_)) * getXi(minimum_index_);
    }

    Eigen::Matrix<double, 4, 4> Joint::getParentGsrTheta(const int &minimum_index_)
    {
        if(!isParent(minimum_index_)) return Eigen::Matrix<double, 4, 4>::Zero();

        if(_parent == nullptr) return getExpXiHatTheta()*_gsr_zero;

        _minimum_index = minimum_index_;
        _parent->_minimum_index = _minimum_index;

        return getParentGstThetaRecursion()*_gsr_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::get_dGsr_dTh(const int &minimum_index_, const int &differentiating_index_)
    {
        if((differentiating_index_ < minimum_index_) || (_index < differentiating_index_)) return Eigen::Matrix<double, 4, 4>::Zero();
        if(!isParent(minimum_index_)) return Eigen::Matrix<double, 4, 4>::Zero();
        if(!isParent(differentiating_index_)) return Eigen::Matrix<double, 4, 4>::Zero();

        _minimum_index = minimum_index_;
        _differentiating_index = differentiating_index_;

        return get_dGsr_dThRecursion() * _gsr_zero;
    }

    Eigen::Matrix<double, 4, 4> Joint::get_dGsr_dThRecursion()
    {
        if(_index == _minimum_index)
        {
            if(_index == _differentiating_index) return _xi_hat * getExpXiHatTheta();

            return getExpXiHatTheta();
        }

        _parent->_minimum_index = _minimum_index;
        _parent->_differentiating_index = _differentiating_index;

        if(_index == _differentiating_index) return _parent->get_dGsr_dThRecursion() * _xi_hat * getExpXiHatTheta();

        return _parent->get_dGsr_dThRecursion() * getExpXiHatTheta();
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

        if(_children.size() == 0) return tree + "(" + std::to_string(_index) + ")" + "\n\n";

        tree += std::to_string(_index);
        tree += " --->\t";

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