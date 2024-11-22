#include "ec_calculator/torque_current_converter.h"

namespace ec_calculator
{
    void TorqueCurrentConverter::setMotorId(const int &motor_id_)
    {
        switch(motor_id_)
        {
            case 1:
                _a = 4.006;
                _b = 0.2176;
                // _b = 0.30-0.02;
                _c = 4.296;
                _d = 0.1809;
                // _d = 0.27-0.02;
                break;
            case 2:
                _a = 4.037;
                _b = 0.1907;
                _c = 4.880;
                _d = 0.2619;
                break;
            case 3:
                _a = 3.362;
                _b = 0.2199;
                _c = 4.394;
                _d = 0.3066;
                break;
            case 4:
                _a = 2.345;
                // _b = 0.2591;
                _b = 0.25;
                _c = 3.441;
                // _d = 0.2558;
                _d = 0.23;
                break;
            case 5:
                _a = 2.356;
                _b = 0.02650;
                _c = 2.804;
                _d = 0.0;
                break;
            case 6:
                _a = 2.943;
                _b = 0.0+0.11;
                _c = 3.728;
                _d = 0.01448+0.11;
                break;
            default:
                break;
        }
    }

    double TorqueCurrentConverter::current2torque(const double &current_)
    {
        if(current_ < _b)
        {
            return _a*(current_+_b);
        }
        if(current_ > _d)
        {
            return _c*(current_-_d);
        }
        return 0.0;
    }

    double TorqueCurrentConverter::torque2current(const double &torque_)
    {
        if(torque_ < 0.0)
        {
            return (1/_a)*torque_-_b;
        }
        if(torque_ > 0.0)
        {
            return (1/_c)*torque_+_d;
        }
        return 0.0;
    }
}