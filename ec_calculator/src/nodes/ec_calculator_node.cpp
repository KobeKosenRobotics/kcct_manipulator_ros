#include "ec_calculator/manipulator.h"

using namespace ec_calculator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EcCalculator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    Manipulator manip;

    while(nh.ok())
    {
        manip.print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}