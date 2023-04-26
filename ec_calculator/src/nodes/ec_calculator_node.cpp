#include "ec_calculator/manipulator.h"
#include "ec_calculator/manipulator_tf_publisher.h"

using namespace ec_calculator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EcCalculator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    Manipulator manip;
    ManipulatorTFPublisher tfPublisher(manip);

    while(nh.ok())
    {
        manip.print();
        tfPublisher.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}