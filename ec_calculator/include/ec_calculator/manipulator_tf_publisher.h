#ifndef MANIPULATOR_TF_PUBLISHER_H

#include "ec_calculator/manipulator.h"

#include <tf/transform_broadcaster.h>

namespace ec_calculator
{
    class ManipulatorTFPublisher
    {
        public:
            ManipulatorTFPublisher(Manipulator& manipulator);
            void publish();
        private:
            Manipulator *_manipulator;
    };
}

#define MANIPULATOR_TF_PUBLISHER_H
#endif