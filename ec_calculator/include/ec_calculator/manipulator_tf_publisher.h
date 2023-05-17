#ifndef EC_CALCULATOR_MANIPULATOR_TF_PUBLISHER_H

#include "ec_calculator/manipulator.h"

#include <iostream>

#include <tf/transform_broadcaster.h>

namespace ec_calculator
{
    class ManipulatorTFPublisher
    {
        public:
            ManipulatorTFPublisher(Manipulator& manipulator);
            void publish();
            void publish(const std::string &base_name_, const std::string &point_name_, const Eigen::Matrix<double, 6, 1> &pose_);
        private:
            Manipulator *_manipulator;
    };
}

#define EC_CALCULATOR_MANIPULATOR_TF_PUBLISHER_H
#endif