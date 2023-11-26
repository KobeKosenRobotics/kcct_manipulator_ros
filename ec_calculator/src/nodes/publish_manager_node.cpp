#include "ec_calculator/wait.h"
#include "ec_calculator/publish_manager.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

using namespace ec_calculator;

Wait wait;
PublishManager pm;

int JOINT_NUM = 15;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_manager_node");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Publisher
    ros::Publisher emergency_stop_pub = nh.advertise<std_msgs::Bool>("emergency_stop", 100);
    ros::Publisher ik_enable_pub = nh.advertise<std_msgs::Bool>("ik_enable", 100);
    ros::Publisher motor_enable_pub = nh.advertise<std_msgs::Bool>("motor_enable", 100);
    ros::Publisher polygon_enable_pub = nh.advertise<std_msgs::Bool>("polygon_enable", 100);
    ros::Publisher simulation_enable_pub = nh.advertise<std_msgs::Bool>("simulation_enable", 100);
    ros::Publisher torque_enable_pub = nh.advertise<std_msgs::Bool>("torque_enable", 100);
    ros::Publisher target_angle_pub = nh.advertise<std_msgs::Float32MultiArray>("target_angle", 100);
    ros::Publisher target_pose_pub = nh.advertise<std_msgs::Float32MultiArray>("target_pose", 100);

    pm.setJointNum(JOINT_NUM);

    int step = 0;

    while(nh.ok())
    {
        switch (step)
        {
        case 0:
            pm.setEmergencyStop(true);
            pm.setMotorEnable(true);
            pm.setPolygonEnable(true);

            if(!wait.isWaiting(5.0)) step++;
            break;

        case 1:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 2:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 3:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 4:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 5:
            pm.setTargetAngle(0.4,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(1.5*10.0)) step++;
            break;

        case 6:
            pm.setTargetAngle(0.4,-1.5,1.2,0.1,0.3,0.1,    -0.5,0.7,0.7,    -0.5,0.7,0.7,    -0.5,0.7,0.7);

            if(!wait.isWaiting(1.5*10.0)) step++;
            break;

        case 10:
            pm.setTargetPose(0.25+0.08,-0.13,-0.92,    -0.6,0,0,    0.1);
            pm.setIKEnable(true);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        case 15:
            pm.setTargetPose(0.25+0.08,-0.13,-0.92,    -0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        case 20:
            pm.setTargetPose(0.25+0.08,-0.13,-0.86,    -0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) step++;
            break;

        case 23:
            pm.setTargetPose(0.25+0.08,-0.13,-0.86,    0,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) step++;
            break;

        case 25:
            pm.setTargetPose(0.25+0.08,0,-0.86,    0,0,0,    0.03);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        case 30:
            pm.setTargetPose(0.25+0.08,0.13,-0.86,    0,0,0,    0.03);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        case 33:
            pm.setTargetPose(0.25+0.08,0.13,-0.86,    0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) step++;
            break;

        case 35:
            pm.setTargetPose(0.25+0.08,0.13,-0.92,    0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) step++;
            break;

        case 40:
            pm.setTargetPose(0.25+0.08,0.13,-0.92,    0.6,0,0,    0.1);

            if(!wait.isWaiting(1.5*10.0)) step++;
            break;

        case 45:
            pm.setTargetAngle(-0.4,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);
            pm.setIKEnable(false);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 50:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 55:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 60:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 65:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.0,0.0,0.0,0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

            if(!wait.isWaiting(10.0)) step++;
            break;

        case 70:
            pm.setEmergencyStop(true);
            pm.setMotorEnable(false);
            break;

        default:
            step++;
            break;
        }
        // switch (step)
        // {
        // case 0:
        //     pm.setEmergencyStop(true);

        //     if(!wait.isWaiting(2.0)) step++;
        //     break;

        // case 5:
        //     pm.setEmergencyStop(false);
        //     pm.setTargetAngle(-0.5,-0.3,0.0,0.1,-1.5,0.1,    -0.5,0.2,0.2,    -0.5,0.2,0.2,    -0.5,0.2,0.2);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 8:
        //     pm.setEmergencyStop(false);
        //     pm.setTargetAngle(-0.5,-0.3,1.5,0.1,-1.2,0.1,    -0.5,0.2,0.2,    -0.5,0.2,0.2,    -0.5,0.2,0.2);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 10:
        //     pm.setTargetPose(0.15,0.12,-1,    0.46,-0,0,    0.1);
        //     pm.setIKEnable(true);

        //     if(!wait.isWaiting(5.0)) step++;
        //     break;

        // case 15:
        //     pm.setTargetPose(0.15,0.12,-1,    0.46,-0,0,    0.05);

        //     if(!wait.isWaiting(5.0)) step++;
        //     break;

        // case 25:
        //     pm.setTargetPose(0.3,0.0,-0.95,    0,-0,0,    0.05);

        //     if(!wait.isWaiting(30.0)) step++;
        //     break;

        // case 30:
        //     pm.setTargetPose(0.15,-0.12,-1,    -0.46,-0,0,    0.05);

        //     if(!wait.isWaiting(30.0)) step++;
        //     break;

        // case 40:
        //     pm.setTargetPose(0.15,-0.12,-1,    -0.46,-0,0,    0.1);

        //     if(!wait.isWaiting(5.0)) step++;
        //     break;

        // case 45:
        //     pm.setTargetAngle(0.5,-0.3,1.5,-0.1,-1.2,-0.1,    -0.5,0.2,0.2,    -0.5,0.2,0.2,    -0.5,0.2,0.2);
        //     pm.setIKEnable(false);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 50:
        //     pm.setTargetAngle(-3.1,-0.3,0.0,0.0,-1.2,0.0,    -1.0,0.0,0.0,    -1.0,0.0,0.0,    -1.0,0.0,0.0);

        //     if(!wait.isWaiting(15.0)) step++;
        //     break;

        // case 55:
        //     pm.setTargetAngle(-3.1,-0.3,0.0,0.0,-1.2,0.0,    1.0,0.3,0.3,    1.0,0.3,0.3,    1.0,0.3,0.3);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 60:
        //     pm.setTargetAngle(-3.1,-0.3,0.0,0.0,-1.2,0.0,    -1.0,0.0,0.0,    -1.0,0.0,0.0,    -1.0,0.0,0.0);

        //     if(!wait.isWaiting(15.0)) step++;
        //     break;

        // default:
        //     step++;
        //     break;
        // }

        emergency_stop_pub.publish(pm.getEmergencyStop());
        ik_enable_pub.publish(pm.getIKEnable());
        motor_enable_pub.publish(pm.getMotorEnable());
        simulation_enable_pub.publish(pm.getSimulationEnable());
        polygon_enable_pub.publish(pm.getPolygonEnable());
        torque_enable_pub.publish(pm.getTorqueEnable());
        target_angle_pub.publish(pm.getTargetAngle());
        target_pose_pub.publish(pm.getTargetPose());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}