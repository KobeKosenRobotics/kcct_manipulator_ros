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
    manip.print();

    int cha = 4, joi = 7;
    Eigen::Matrix<bool, 4, 7> cha_ma;
    cha_ma <<
    // 1  2  3  4  5  6  7  8  9 10
    1, 1, 1, 0, 0, 0, 0,
    1, 1, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1, 1, 0,
    1, 0, 0, 0, 1, 0, 1;
    Eigen::Matrix<double, 3, 7> joi_po;
    joi_po <<
    1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 3, 7> tol_po;
    tol_po <<
    1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 3, 7> tra;
    Eigen::Matrix<double, 3, 7> rot;
    rot <<
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1;

    model.changeModel(cha, joi, cha_ma, joi_po, tol_po, tra, rot);

    manip.init(&model);
    manip.printTree();
    manip.print();

    while(nh.ok())
    {
        tfPublisher.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}