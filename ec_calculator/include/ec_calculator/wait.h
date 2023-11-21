#ifndef CLASS_WAIT_H
#define CLASS_WAIT_H

#include <ros/ros.h>
#include <iostream>

namespace ec_calculator
{
    class Wait
    {
    private:
        ros::Time _start, _end;
        bool _is_first = true;
        ros::Duration _duration;
        double _duration_seconds;

    public:
        bool isWaiting(const double &seconds_);
        void reset();
    };
}

#endif