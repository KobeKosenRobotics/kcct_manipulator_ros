#ifndef JOINT_H

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ec_calculator
{
    class Joint
    {
        private:
            int _index = 0;
            std::string _name = "";

            // Family
            Joint* _parent = nullptr;
            std::vector<Joint*> _children { nullptr };

            /*
            // Parameter
            Eigen::Matrix<double, 3, 1> _q, _v, _w;
            Eigen::Matrix<double, 6, 1> _xi;
            Eigen::Matrix<double, 4, 4> _gsj_zero;

            double _theta, _cos_theta, _sin_theta, _v_theta;

            // Matrix
            Eigen::Matrix<double, 3, 3> _exp_w_hat_theta;
            Eigen::Matrix<double, 4, 4> _exp_xi_hat_theta;
            Eigen::Matrix<double, 6, 1> _xi_dagger;

            int _minimum_joint;
            */

        public:

            // Constructor
            Joint();

            // Initialize
            void init(const int index, const std::string name);

            // Chain Management Functions
            void setParent(Joint &parent);
            bool addChild(Joint &child);
            void clearChildren();

            // Properties
            int getIndex();
            std::string getName();

            // Debug
            std::string getChildrenList();
            std::string getChildrenList(const int tab);

            /*
            // Constructor
            Joint();

            // Family
            void setJoint(const int &joint_index_);
            void setJointProperty();
                Eigen::Matrix<double, 3, 1> getQ();
                Eigen::Matrix<double, 6, 1> getXi();
                Eigen::Matrix<double, 4, 4> getGsjZero();
            void setParent(Joint *parent_joint_);
            void setChildren(Joint *children_joint_);
            void printJoint();

            // Theta
            void updateTheta(const double &theta_);

            // Matrix
            Eigen::Matrix<double, 4, 4> getExpXiHatTheta();
                Eigen::Matrix<double, 3, 3> getExpWHatTheta();

            Eigen::Matrix<double, 4, 4> getGsjTheta();
                Eigen::Matrix<double, 4, 4> getGsjThetaRecursion();
            Eigen::Matrix<double, 4, 4> getChildrenExpXiHatTheta(const int  &minimum_joint_);
                Eigen::Matrix<double, 4, 4> getChildrenExpXiHatThetaRecursion();

            Eigen::Matrix<double, 6, 1> getXiDagger(const int &chain_, const int &joint_);
            */
    };
}

#define JOINT_H
#endif