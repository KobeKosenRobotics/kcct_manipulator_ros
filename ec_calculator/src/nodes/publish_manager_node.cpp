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

    int cup = 0;

    while(nh.ok())
    {
        /* Cup */
        switch (cup)
        {
        case 0:
            pm.setEmergencyStop(true);
            pm.setMotorEnable(true);
            pm.setPolygonEnable(true);

            if(!wait.isWaiting(5.0)) cup++;
            break;

        case 1:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 2:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 3:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 4:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 5:
            pm.setTargetAngle(0.4,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(1.5*10.0)) cup++;
            break;

        case 6:
            pm.setTargetAngle(0.4,-1.5,1.2,0.1,0.3,0.1,    -0.5,0.7,0.7,    -0.5,0.7,0.7,    -0.5,0.7,0.7);

            if(!wait.isWaiting(1.5*10.0)) cup++;
            break;

        case 10:
            pm.setTargetPolygon(0.25+0.08,-0.13,-0.92,    -0.6,0,0,    0.1);
            pm.setIKEnable(true);

            if(!wait.isWaiting(1.5*15.0)) cup++;
            break;

        case 15:
            pm.setTargetPolygon(0.25+0.08,-0.13,-0.92,    -0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*15.0)) cup++;
            break;

        case 20:
            pm.setTargetPolygon(0.25+0.08,-0.13,-0.86,    -0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) cup++;
            break;

        case 23:
            pm.setTargetPolygon(0.25+0.08,-0.13,-0.86,    0,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) cup++;
            break;

        case 25:
            pm.setTargetPolygon(0.25+0.08,0,-0.86,    0,0,0,    0.03);

            if(!wait.isWaiting(1.5*15.0)) cup++;
            break;

        case 30:
            pm.setTargetPolygon(0.25+0.08,0.13,-0.86,    0,0,0,    0.03);

            if(!wait.isWaiting(1.5*15.0)) cup++;
            break;

        case 33:
            pm.setTargetPolygon(0.25+0.08,0.13,-0.86,    0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) cup++;
            break;

        case 35:
            pm.setTargetPolygon(0.25+0.08,0.13,-0.92,    0.6,0,0,    0.03);

            if(!wait.isWaiting(1.5*5.0)) cup++;
            break;

        case 40:
            pm.setTargetPolygon(0.25+0.08,0.13,-0.92,    0.6,0,0,    0.1);

            if(!wait.isWaiting(1.5*10.0)) cup++;
            break;

        case 45:
            pm.setTargetAngle(-0.4,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);
            pm.setIKEnable(false);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 50:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 55:
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 60:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 65:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.0,0.0,0.0,0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

            if(!wait.isWaiting(10.0)) cup++;
            break;

        case 70:
            pm.setEmergencyStop(true);
            pm.setMotorEnable(false);
            break;

        default:
            if(cup >= 1000) break;
            cup++;
            break;
        }






        /* Pancake *//*
        switch (step)
        {
        case 0:
            pm.setEmergencyStop(true);
            // pm.setMotorEnable(true);

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

        case 6: // grasp
            pm.setTargetAngle(0.739,-0.540,0.420,-0.669,-1.20,0.0638,    0.420,0.270,-0.331,    0.529,0.278,-0.739,    0.439,0.316,-0.437);

            if(!wait.isWaiting(1.5*20.0)) step++;
            break;

        case 10:
            pm.setTargetAngle(0.439,-0.940,0.620,-0.469,-1.50,0.0638,    0.420,0.270,-0.331,    0.529,0.278,-0.739,    0.439,0.316,-0.437);

            if(!wait.isWaiting(1.5*10.0)) step++;
            break;

        case 15:
            pm.setTargetAngle(0.050,-0.940,0.620,-0.050,-1.50,0.050,    0.420,0.270,-0.331,    0.529,0.278,-0.739,    0.439,0.316,-0.437);

            if(!wait.isWaiting(1.5*10.0)) step++;
            break;

        case 20:
            pm.setTargetAngle(0.0261,-1.06,1.28,-0.0376,-1.071,0.0359,    0.420,0.270,-0.331,    0.529,0.278,-0.739,    0.439,0.316,-0.437);

            if(!wait.isWaiting(1.5*10.0)) step++;
            break;

        case 25:
            pm.setTargetPose(0,5,    0.283,-0.00297,-0.678,    0.000595,-0.899,0.0135);
            pm.setIKEnable(true);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        case 30:
            pm.setTargetPose(0,5,    0.383,-0.00297,-0.678,    0.000595,-0.899,0.0135);
            pm.setIKEnable(true);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        case 35:
            pm.setTargetPose(0,5,    0.383,-0.00297,-0.598,    0.000595,-0.899,0.0135);
            pm.setIKEnable(true);

            if(!wait.isWaiting(1.5*15.0)) step++;
            break;

        // case 15:
        //     pm.setTargetPose(0,5,    0.240,-0.024,-0.594,    0.147,-1.24,-0.229);

        //     if(!wait.isWaiting(1.5*15.0)) step++;
        //     break;

        // case 20:
        //     pm.setTargetPolygon(0.25+0.08,-0.13,-0.86,    -0.6,0,0,    0.03);

        //     if(!wait.isWaiting(1.5*5.0)) step++;
        //     break;

        // case 23:
        //     pm.setTargetPolygon(0.25+0.08,-0.13,-0.86,    0,0,0,    0.03);

        //     if(!wait.isWaiting(1.5*5.0)) step++;
        //     break;

        // case 25:
        //     pm.setTargetPolygon(0.25+0.08,0,-0.86,    0,0,0,    0.03);

        //     if(!wait.isWaiting(1.5*15.0)) step++;
        //     break;

        // case 30:
        //     pm.setTargetPolygon(0.25+0.08,0.13,-0.86,    0,0,0,    0.03);

        //     if(!wait.isWaiting(1.5*15.0)) step++;
        //     break;

        // case 33:
        //     pm.setTargetPolygon(0.25+0.08,0.13,-0.86,    0.6,0,0,    0.03);

        //     if(!wait.isWaiting(1.5*5.0)) step++;
        //     break;

        // case 35:
        //     pm.setTargetPolygon(0.25+0.08,0.13,-0.92,    0.6,0,0,    0.03);

        //     if(!wait.isWaiting(1.5*5.0)) step++;
        //     break;

        // case 40:
        //     pm.setTargetPolygon(0.25+0.08,0.13,-0.92,    0.6,0,0,    0.1);

        //     if(!wait.isWaiting(1.5*10.0)) step++;
        //     break;

        // case 45:
        //     pm.setTargetAngle(-0.4,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);
        //     pm.setIKEnable(false);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 50:
        //     pm.setTargetAngle(0.0,-1.5,1.5,0.0,0.0,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 55:
        //     pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    -1.5,0.1,0.2,    -1.5,0.1,0.2,    -1.5,0.1,0.2);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 60:
        //     pm.setEmergencyStop(false);
        //     pm.setTargetAngle(0.0,-1.5,1.5,0.0,1.5,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 65:
        //     pm.setEmergencyStop(false);
        //     pm.setTargetAngle(0.0,0.0,0.0,0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0,    0.0,0.0,0.0);

        //     if(!wait.isWaiting(10.0)) step++;
        //     break;

        // case 70:
        //     pm.setEmergencyStop(true);
        //     pm.setMotorEnable(false);
        //     break;

        default:
            step++;
            break;
        }
        */

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