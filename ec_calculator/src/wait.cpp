#include "ec_calculator/wait.h"

namespace ec_calculator
{
    bool Wait::isWaiting(const double &seconds_)
    {
        if(_is_first)
        {
            _start = ros::Time::now();
            _is_first = false;
        }

        _end = ros::Time::now();

        _duration = _end-_start;

        _duration_seconds = _duration.toSec();

        // std::cout << _duration_seconds << std::endl;

        if(_duration_seconds < seconds_)
        {
            return true;
        }

        _is_first = true;
        return false;
    }

    void Wait::reset()
    {
        _is_first = true;
    }
}