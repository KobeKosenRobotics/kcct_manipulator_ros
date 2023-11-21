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

            if(!wait.isWaiting(5.0))
            {
                step++;
            }

            break;

        case 5:
            pm.setEmergencyStop(false);
            pm.setTargetAngle(0.1, 0.2, 0.3);

            if(!wait.isWaiting(5.0))
            {
                step++;
            }
            break;

        default:
            step++;
            break;
        }

        emergency_stop_pub.publish(pm.getEmergencyStop());
        ik_enable_pub.publish(pm.getIKEnable());
        motor_enable_pub.publish(pm.getMotorEnable());
        simulation_enable_pub.publish(pm.getSimulationEnable());
        torque_enable_pub.publish(pm.getTorqueEnable());
        target_angle_pub.publish(pm.getTargetAngle());
        target_pose_pub.publish(pm.getTargetPose());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}