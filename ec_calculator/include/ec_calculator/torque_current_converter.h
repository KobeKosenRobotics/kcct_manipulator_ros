#ifndef TORQUE_CURRENT_CONVERTER_H
#define TORQUE_CURRENT_CONVERTER_H

#include <iostream>


namespace ec_calculator
{
    class TorqueCurrentConverter
    {
        private:
            double _a = 0.0;    // negative slope
            double _b = 0.0;    // negative current intercept
            double _c = 0.0;    // positive slope
            double _d = 0.0;    // positive current intercept
        public:
            void setMotorId(const int &motor_id_);
            double current2torque(const double &current_);
            double torque2current(const double &torque_);
    };
}

#endif