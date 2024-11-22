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
std_msgs::Float32MultiArray target_current;

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

void current_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(manip.getMotorEnable())
    {
        manip.updateCurrent(EigenUtility.array2Matrix(msg->data));
    }
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
    std::cout << "gain updated" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ec_calculator_node");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher target_angular_velocity_pub = nh.advertise<std_msgs::Float32MultiArray>("calculator/target_angular_velocity", 100);
    ros::Publisher target_current_pub = nh.advertise<std_msgs::Float32MultiArray>("calculator/target_current", 100);

    // Subscriber
    ros::Subscriber emergency_stop_sub = nh.subscribe<std_msgs::Bool>("user/emergency_stop", 100, emergency_stop_cb);
    ros::Subscriber ik_enable_sub = nh.subscribe<std_msgs::Bool>("user/ik_enable", 10, ik_enable_cb);
    ros::Subscriber motor_enable_sub = nh.subscribe<std_msgs::Bool>("user/motor_enable", 10, motor_enable_cb);
    ros::Subscriber polygon_enable_sub = nh.subscribe<std_msgs::Bool>("user/polygon_enable", 10, polygon_enable_cb);
    ros::Subscriber simulation_enable_sub = nh.subscribe<std_msgs::Bool>("user/simulation_enable", 10, simulation_enable_cb);
    ros::Subscriber torque_enable_sub = nh.subscribe<std_msgs::Bool>("user/torque_enable", 10, torque_enable_cb);
    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("motor/angle", 10, angle_cb);
    ros::Subscriber angular_velocity_sub = nh.subscribe<std_msgs::Float32MultiArray>("motor/angular_velocity", 10, angular_velocity_cb);
    ros::Subscriber current_sub = nh.subscribe<std_msgs::Float32MultiArray>("motor/current", 10, current_cb);
    ros::Subscriber target_angle_sub = nh.subscribe<std_msgs::Float32MultiArray>("user/target_angle", 10, target_angle_cb);
    ros::Subscriber target_pose_sub = nh.subscribe<std_msgs::Float32MultiArray>("user/target_pose", 10, target_pose_cb);
    target_pose.data.resize(2+6);   // 2: start_joint, end_joint, 6: 3position, 3orientation
    ros::Subscriber gains_sub = nh.subscribe<std_msgs::Float32MultiArray>("user/gains", 10, gains_cb);

    manip.init(&model);
    manip.printTree();
    target_angular_velocity.data.resize(manip.getJointNum());
    target_current.data.resize(manip.getJointNum());
    target_angle.data.resize(manip.getJointNum());

    while(nh.ok())
    {
        tfPublisher.publish();

        for(int i = 0; i < manip.getInverseKinematicsNum(); i++)
        {
            tfPublisher.publish("manipulator_base_link", "TargetPose"+std::to_string(i), manip.getTargetPose(i));
            tfPublisher.publish("manipulator_base_link", "MidTargetPose"+std::to_string(i), manip.getMidPose(i));
        }

        target_angular_velocity.data = EigenUtility.matrix2Array(manip.getAngularVelocity());
        target_current.data = EigenUtility.matrix2Array(manip.getCurrent());
        target_angular_velocity_pub.publish(target_angular_velocity);
        target_current_pub.publish(target_current);

        manip.print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}