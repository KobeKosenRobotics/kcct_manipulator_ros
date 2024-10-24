#include "ec_calculator/model.h"
#include "ec_calculator/manipulator.h"
#include "ec_calculator/manipulator_tf_publisher.h"
#include "ec_calculator/eigenUtility.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

using namespace ec_calculator;

Manipulator manip;
ManipulatorTFPublisher tfPublisher(manip);
Model model;

// Publisher
std_msgs::Float32MultiArray target_angular_velocity;
std_msgs::Float32MultiArray target_torque;

// Subscriber
std_msgs::Bool emergency_stop;
std_msgs::Bool ik_enable;
std_msgs::Bool motor_enable;
std_msgs::Bool polygon_enable;
std_msgs::Bool simulation_enable;
std_msgs::Bool torque_enable;

std_msgs::Float32MultiArray angle;
std_msgs::Float32MultiArray angular_velocity;
std_msgs::Float32MultiArray angular_acceleration;
std_msgs::Float32MultiArray torque;
std_msgs::Float32MultiArray current;

std_msgs::Float32MultiArray target_angle;
std_msgs::Float32MultiArray target_pose;    // 2: start_joint, end_joint, 6: 3position, 3orientation

void emergency_stop_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setEmergencyStop(msg->data);
}

void ik_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setIKEnable(msg->data);
}

void motor_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setMotorEnable(msg->data);
}

void polygon_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setPolygonEnable(msg->data);
}

void simulation_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setSimulationEnable(msg->data);
}

void torque_enable_cb(std_msgs::Bool::ConstPtr msg)
{
    manip.setTorqueEnable(msg->data);
}

void angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(manip.getMotorEnable())
    {
        manip.updateAngle(EigenUtility.array2Matrix(msg->data));
    }
}

void angular_velocity_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(manip.getMotorEnable())
    {
        manip.updateAngularVelocity(EigenUtility.array2Matrix(msg->data));
        manip.updateAngularAcceleration(EigenUtility.array2Matrix(msg->data));
    }
}

void torque_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    manip.updateTorque(EigenUtility.array2Matrix(msg->data));
}

void current_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    manip.updateCurrent(EigenUtility.array2Matrix(msg->data));
}

void target_angle_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    manip.setTargetAngle(EigenUtility.array2Matrix(msg->data));
}

void target_pose_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(manip.getPolygonEnable())
    {
        manip.setTargetPolygon(EigenUtility.array2Matrix(msg->data));
        return;
    }

    manip.setTargetPose(EigenUtility.array2Matrix(msg->data));
}

void gains_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    manip.setGains(EigenUtility.array2Matrix(msg->data));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ec_calculator_node");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher target_angular_velocity_pub = nh.advertise<std_msgs::Float32MultiArray>("target_angular_velocity", 100);
    ros::Publisher target_torque_pub = nh.advertise<std_msgs::Float32MultiArray>("target_torque", 100);

    // Subscriber
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("emergency_stop", 100, emergency_stop_cb);
    ros::Subscriber ik_enable_sub = nh.subscribe<std_msgs::Bool>("ik_enable", 10, ik_enable_cb);
    ros::Subscriber motor_enable_sub = nh.subscribe<std_msgs::Bool>("motor_enable", 10, motor_enable_cb);
    ros::Subscriber polygon_enable_sub = nh.subscribe<std_msgs::Bool>("polygon_enable", 10, polygon_enable_cb);
    ros::Subscriber simulation_enable_sub = nh.subscribe<std_msgs::Bool>("simulation_enable", 10, simulation_enable_cb);
    ros::Subscriber torque_enable_sub = nh.subscribe<std_msgs::Bool>("torque_enable", 10, torque_enable_cb);
    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("angle", 10, angle_cb);
    ros::Subscriber angular_velocity_sub = nh.subscribe<std_msgs::Float32MultiArray>("angular_velocity", 10, angular_velocity_cb);
    ros::Subscriber torque_sub = nh.subscribe<std_msgs::Float32MultiArray>("torque", 10, torque_cb);
    ros::Subscriber current_sub = nh.subscribe<std_msgs::Float32MultiArray>("current", 10, current_cb);
    ros::Subscriber target_angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_angle", 10, target_angle_cb);
    ros::Subscriber target_pose_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_pose", 10, target_pose_cb);
    target_pose.data.resize(2+6);   // 2: start_joint, end_joint, 6: 3position, 3orientation
    ros::Subscriber gains_sub = nh.subscribe<std_msgs::Float32MultiArray>("gains", 10, gains_cb);

    manip.init(&model);
    manip.printTree();
    target_angular_velocity.data.resize(manip.getJointNum());
    target_torque.data.resize(manip.getJointNum());
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
    // target_angular_velocity.data.resize(manip.getJointNum());
    // target_torque.data.resize(manip.getJointNum());
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
    // target_angular_velocity.data.resize(manip.getJointNum());
    // target_torque.data.resize(manip.getJointNum());
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
    // target_angular_velocity.data.resize(manip.getJointNum());
    // target_torque.data.resize(manip.getJointNum());
    // target_angle.data.resize(manip.getJointNum());

    while(nh.ok())
    {
        tfPublisher.publish();

        for(int i = 0; i < manip.getInverseKinematicsNum(); i++)
        {
            tfPublisher.publish("manipulator_base_link", "TargetPose"+std::to_string(i), manip.getTargetPose(i));
            tfPublisher.publish("manipulator_base_link", "MidTargetPose"+std::to_string(i), manip.getMidPose(i));
        }

        target_angular_velocity.data = EigenUtility.matrix2Array(manip.getAngularVelocity());
        target_torque.data = EigenUtility.matrix2Array(manip.getCurrent());
        target_angular_velocity_pub.publish(target_angular_velocity);
        target_torque_pub.publish(target_torque);

        manip.print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}