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
    1, 1, 1, 1, 0, 0, 0,
    1, 1, 1, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 3, 7> joi_po;
    joi_po <<
    1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 2, 3,
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
    Eigen::Matrix<double, 7, 7> a2a_ga;
    a2a_ga.setIdentity();
    a2a_ga *= 2.0;
    double ec_ga = 2.0;

    model.changeModel(cha, joi, cha_ma, joi_po, tol_po, tra, rot, a2a_ga, ec_ga);

    manip.init(&model);
    manip.printTree();
    manip.print();

    Eigen::Matrix<double, 7, 1> ta_an;
    ta_an.setConstant(0.5);

    while(nh.ok())
    {
        tfPublisher.publish();

        manip.angularVelocity2Angle(manip.getAngularVelocityByAngle(ta_an));    // callBack(){manip.setAngle(msg);} while(){pub.publish(manip.getAngularVelocity());}

        // manip.print();
        tfPublisher.publish("manipulator_base_link", "pose0", manip.getPose(3));
        // std::cout << manip.getPose(3).transpose() << std::endl <<std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}