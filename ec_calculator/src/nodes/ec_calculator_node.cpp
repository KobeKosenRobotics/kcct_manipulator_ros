#include "ec_calculator/model.h"
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
    Model model;

    manip.init(&model);
    manip.printTree();

    Eigen::Matrix<bool, 4, 7> chain;
    chain <<
    // 1  2  3  4  5  6  7  8  9 10
    1, 1, 1, 0, 0, 0, 0,
    1, 1, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1, 1, 0,
    1, 0, 0, 0, 1, 0, 1;

    model.changeModel(4, 7, chain);

    manip.init(&model);
    manip.printTree();

    while(nh.ok())
    {
        tfPublisher.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}