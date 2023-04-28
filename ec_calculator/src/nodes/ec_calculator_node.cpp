#ifndef JOINT_NUM
#define JOINT_NUM 10
#endif

#ifndef CHAIN_NUM
#define CHAIN_NUM 3
#endif

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

    manip.init();

    Eigen::Matrix<bool, CHAIN_NUM, JOINT_NUM> chain_matrix;
    chain_matrix <<
    // 1  2  3  4  5  6  7  8  9 10
    1, 1, 1, 0, 0, 1, 0, 0, 1, 0,
    1, 1, 0, 1, 0, 0, 1, 0, 0, 1,
    1, 1, 0, 0, 1, 0, 0, 1, 0, 0;
    manip.setChainMatrix(chain_matrix);

    manip.printTree();

    while(nh.ok())
    {
        std::cout << model.getJointNum() << std::endl;
        tfPublisher.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}