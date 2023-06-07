#include "ec_calculator/model.h"
#include "ec_calculator/manipulator.h"
#include "ec_calculator/manipulator_tf_publisher.h"
#include "ec_calculator/eigenUtility.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

using namespace ec_calculator;

Manipulator manip;
ManipulatorTFPublisher tfPublisher(manip);
Model model;

ros::Time start, end;
double sum = 0;
int cycle = 0;

// Publisher
std_msgs::Float32MultiArray angular_velocity;

// Subscriber
std_msgs::Bool ec_enable;
std_msgs::Bool emergency_stop;
std_msgs::Bool motor_enable;
std_msgs::Float32MultiArray target_angle;
std_msgs::Float32MultiArray target_pose;    // 2: start_joint, end_joint, 6: 3position, 3orientation

void ec_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setECEnable(msg->data);
}

void emergency_stop_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setEmergencyStop(msg->data);
}

void motor_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setMotorEnable(msg->data);
}

void target_angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    manip.setTargetAngle(EigenUtility.array2Matrix(msg->data));
}

void target_pose_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    manip.setTargetPose(EigenUtility.array2Matrix(msg->data));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ECCalculator");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher angular_velocity_pub = nh.advertise<std_msgs::Float32MultiArray>("angular_velocity", 100);

    // Subscriber
    ros::Subscriber ec_enable_sub = nh.subscribe<std_msgs::Bool>("ec_enable", 10, ec_enable_cb);
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("emergency_stop", 100, emergency_stop_cb);
    ros::Subscriber motor_enable_sub = nh.subscribe<std_msgs::Bool>("motor_enable", 10, motor_enable_cb);
    ros::Subscriber target_angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_angle", 10, target_angle_cb);
    ros::Subscriber target_pose_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_pose", 10, target_pose_cb);
    target_pose.data.resize(2+6);   // 2: start_joint, end_joint, 6: 3position, 3orientation

    manip.init(&model);
    manip.printTree();
    angular_velocity.data.resize(manip.getJointNum());
    target_angle.data.resize(manip.getJointNum());

    // /* New naviT(oo)n */
    // int cha = 1, joi = 8;
    // Eigen::Matrix<bool, 1, 8> cha_ma;
    // for(int i = 0; i < joi; i++)
    // {
    //     cha_ma(0, i) = 1;
    // }
    // Eigen::Matrix<double, 3, 9> joi_po;
    // joi_po <<
    //     0, 0, 0, 0.01, 0.030,  0.030, 0,  0    , 0,
    //     0, 0, 0, 0   , 0.264, -0.258, 0,  0    , 0,
    //     0, 0, 0, 0   , 0    ,  0,     0, -0.123, 0;
    // Eigen::Matrix<double, 3, 8> tra;
    // tra.setZero();
    // tra(0, 0) = 1;
    // tra(1, 1) = 1;
    // tra(2, 2) = 1;
    // Eigen::Matrix<double, 3, 8> rot;
    // rot <<
    //     0, 0, 0,  0,  0,  0, 1,  0,
    //     0, 0, 0,  0,  0, -1, 0,  0,
    //     0, 0, 0, -1, -1,  0, 0, -1;
    // Eigen::Matrix<double, 8, 8> a2a_ga;
    // a2a_ga.setIdentity();
    // double ec_ga = 1;

    // model.changeModel(cha, joi, cha_ma, joi_po, tra, rot, a2a_ga, ec_ga);

    // manip.init(&model);
    // manip.printTree();
    // manip.print();
    // angular_velocity.data.resize(manip.getJointNum());
    // target_angle.data.resize(manip.getJointNum());

    // /* Serial 30-DOF */
    // int cha = 1, joi = 30;
    // Eigen::Matrix<bool, 1, 30> cha_ma;
    // for(int i = 0; i < joi; i++)
    // {
    //     cha_ma(0, i) = 1;
    // }
    // Eigen::Matrix<double, 3, 31> joi_po;
    // joi_po.setZero();
    // for(int i = 4; i < (cha+joi); i++)
    // {
    //     joi_po(2,i) = 0.2;
    // }
    // Eigen::Matrix<double, 3, 30> tra;
    // tra.setZero();
    // tra(0, 0) = 1;
    // tra(1, 1) = 1;
    // tra(2, 2) = 1;
    // Eigen::Matrix<double, 3, 30> rot;
    // rot.setZero();
    // for(int i = 3; i < joi; i++)
    // {
    //     rot(i%3, i) = 1;
    // }
    // Eigen::Matrix<double, 30, 30> a2a_ga;
    // a2a_ga.setIdentity();
    // double ec_ga = 2;

    // model.changeModel(cha, joi, cha_ma, joi_po, tra, rot, a2a_ga, ec_ga);

    // manip.init(&model);
    // manip.printTree();
    // manip.print();
    // angular_velocity.data.resize(manip.getJointNum());
    // target_angle.data.resize(manip.getJointNum());

    // /* Chain 30-DOF */
    // int cha = 3, joi = 30;
    // Eigen::Matrix<bool, 3, 30> cha_ma;
    // cha_ma <<
    //     //             5                         14             19          23                29
    //     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
    //     1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1;
    // Eigen::Matrix<double, 3, 33> joi_po;
    // joi_po.setZero();
    // for(int i = 4; i < (cha+joi); i++)
    // {
    //     joi_po(2,i) = 0.2;
    // }
    // joi_po(0, 15) = 0.2;
    // joi_po(0, 24) = 0.2;
    // Eigen::Matrix<double, 3, 30> tra;
    // tra.setZero();
    // tra(0, 0) = 1;
    // tra(1, 1) = 1;
    // tra(2, 2) = 1;
    // Eigen::Matrix<double, 3, 30> rot;
    // rot.setZero();
    // for(int i = 3; i < joi; i++)
    // {
    //     rot(i%3, i) = 1;
    // }
    // Eigen::Matrix<double, 30, 30> a2a_ga;
    // a2a_ga.setIdentity();
    // double ec_ga = 2;

    // model.changeTorqueControlEnable(false);
    // model.changeModel(cha, joi, cha_ma, joi_po, tra, rot, a2a_ga, ec_ga);

    // manip.init(&model);
    // manip.printTree();
    // manip.print();
    // angular_velocity.data.resize(manip.getJointNum());
    // target_angle.data.resize(manip.getJointNum());

    while(nh.ok())
    {
        tfPublisher.publish();
        for(int i = 0; i < manip.getInverseKinematicsNum(); i++)
        {
            tfPublisher.publish("manipulator_base_link", "TargetPose"+std::to_string(i), manip.getTargetPose(i));
            tfPublisher.publish("manipulator_base_link", "MidTargetPose"+std::to_string(i), manip.getMidPose(i));
        }

        manip.angularVelocity2Angle(manip.getAngularVelocity());    // callBack(){manip.setAngle(msg);} while(){pub.publish(manip.getAngularVelocity());}

        angular_velocity.data = EigenUtility.matrix2Array(manip.getAngularVelocity());
        angular_velocity_pub.publish(angular_velocity);

        manip.print();

        /* Time Measure */
        start = ros::Time::now();
        manip.getMf();
        manip.getCf();
        manip.getNf();
        end = ros::Time::now();
        sum += (end - start).toSec();
        cycle++;
        std::cout << cycle << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "time: " << sum/cycle << "\tcycle: " << cycle << std::endl << std::endl;

    return 0;
}