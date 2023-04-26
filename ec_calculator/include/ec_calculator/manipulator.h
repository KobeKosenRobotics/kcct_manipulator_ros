#ifndef MANIPULATOR_H

#include "joint.h"

namespace ec_calculator
{
    class Manipulator
    {
        private:
            Joint _joint;

        public:
            void print();
    };
}

#define MANIPULATOR_H
#endif