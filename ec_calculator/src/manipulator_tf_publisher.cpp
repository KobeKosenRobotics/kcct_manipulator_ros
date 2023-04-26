#include "ec_calculator/manipulator_tf_publisher.h"

namespace ec_calculator
{
    ManipulatorTFPublisher::ManipulatorTFPublisher(Manipulator& manipulator)
    {
        _manipulator = &manipulator;
    }

    void ManipulatorTFPublisher::publish()
    {
    }
}