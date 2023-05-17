#ifndef EC_CALCULATOR_JOINT_H

#include "model.h"

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

            // Parameter
            Eigen::Matrix<double, 3, 1> _q;         // joint position
            Eigen::Matrix<double, 3, 1> _v;         // translation axis
            Eigen::Matrix<double, 3, 1> _w;         // rotation axis
            Eigen::Matrix<double, 3, 1> _lp;        // joint position link (parent)
            Eigen::Matrix<double, 6, 1> _xi;        // twist
            Eigen::Matrix<double, 4, 4> _gst_zero;  // initial homogeneous transformation matrix

            // Visualize
            Eigen::Matrix<double, 6, 1> _visual_data;

            // save temporarily
            double _theta = 0.0, _cos_theta, _sin_theta, _v_theta;

            // Matrix
            Eigen::Matrix<double, 3, 3> _exp_w_hat_theta;   // rotation matrix
            Eigen::Matrix<double, 4, 4> _exp_xi_hat_theta;  // homogeneous transformation matrix
            Eigen::Matrix<double, 6, 1> _xi_dagger;         // a column of Jacobian body matrix

            // save temporarily
            int _minimum_index;

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
            std::string getParentName();
            int getNumOfParentGenerations();
            bool isTipJoint();

            // Parameter Setters
            void setParameters(Model *model_);
                void clearParameters();
                void setQ(const Eigen::Matrix<double, 3, 1> &joint_position_link_);
                void setV(const Eigen::Matrix<double, 3, 1> &translation_axis_);
                void setW(const Eigen::Matrix<double, 3, 1> &rotation_axis_);
                void setXi();
                void setGstZero();

            // Visualize
            double getVisualData(const int &index_);
                void setVisualData();
                Eigen::Matrix<double, 6, 1> getVisualData();

            // Forward Kinematics
            void updateTheta(const double &theta_);
            Eigen::Matrix<double, 4, 4> getExpXiHatTheta();                     // homogeneous transformation matrix (joint)
                Eigen::Matrix<double, 3, 3> getExpWHatTheta();                  // rotation matrix

            Eigen::Matrix<double, 4, 4> getGstTheta();                          // homogeneous transformation matrix (tool)
                Eigen::Matrix<double, 4, 4> getGstThetaRecursion();

            // Inverse Kinematics
            Eigen::Matrix<double, 6, 1> getXiDagger(const int &minimum_index_); // a column of Jacobian body matrix
                bool isParent(const int &parent_index_);
                Eigen::Matrix<double, 6, 1> getXi(const int &parent_index_);
                Eigen::Matrix<double, 4, 4> getParentGstTheta(const int &minimum_index_);
                    Eigen::Matrix<double, 4, 4> getParentGstThetaRecursion();

            // Debug
            std::string getChildrenList();
            std::string getChildrenList(const int tab);
    };
}

#define EC_CALCULATOR_JOINT_H
#endif