#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);
const int JOINT_NUM = 1;

class Torque2Current
{
    private:
        // double _target_torque;
        // double _angular_velocity;
        // double _current;
        // double _target_current;
        double _a = 1.5294117647058838e-06;
        double _b = 0.00015931372549019438;
        double _c = -0.06803921568627373;
        double _d = -1.5294117647058828e-06;
        double _e = 0.00015931372549019598;
        double _f = -1.0419607843137264;
        double _g = -1.11;
    public:
        double current2torque(const double &current_)
        {
            double torque_;

            if(current_ >= 0.0)
            {
                torque_ = _a*current_*current_ + _b*current_ + _c;
            }
            else
            {
                torque_ = _d*current_*current_ + _e*current_ + _f;
            }

            return torque_;
        };
        double torque2current(const double &torque_)
        {
            double current_;

            if(torque_ >= 0.0)
            {
                current_ = (-_b+sqrt(_b*_b-4*_a*(_c-torque_)))/(2.0*_a);
            }
            else
            {
                current_ = (-_e+sqrt(_e*_e-4*_d*(_f-torque_)))/(2.0*_d);
            }

            return current_;
        }
};

// Publisher
std_msgs::Float32MultiArray torque;
std_msgs::Float32MultiArray target_current;

// Subscriber
std_msgs::Float32MultiArray angular_velocity;
std_msgs::Float32MultiArray current;
std_msgs::Float32MultiArray target_torque;

void angular_velocity_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    angular_velocity = *msg;
}

void current_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    current = *msg;
}

void target_torque_cb(std_msgs::Float32MultiArray::ConstPtr msg)
{
    target_torque = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "torque2current_node");
    ros::NodeHandle nh;
    double rate = 100.0;
    ros::Rate loop_rate(rate);

    Torque2Current torque2current;

    // Publisher
    ros::Publisher torque_pub = nh.advertise<std_msgs::Float32MultiArray>("torque", 100);
    torque.data.resize(JOINT_NUM);
    ros::Publisher target_current_pub = nh.advertise<std_msgs::Float32MultiArray>("target_current", 100);
    target_current.data.resize(JOINT_NUM);

    // Subscriber
    ros::Subscriber angular_velocity_sub = nh.subscribe<std_msgs::Float32MultiArray>("angular_velocity", 100, angular_velocity_cb);
    angular_velocity.data.resize(JOINT_NUM);
    ros::Subscriber current_sub = nh.subscribe<std_msgs::Float32MultiArray>("current", 100, current_cb);
    current.data.resize(JOINT_NUM);
    ros::Subscriber target_torque_sub = nh.subscribe<std_msgs::Float32MultiArray>("target_torque", 100, target_torque_cb);
    target_torque.data.resize(JOINT_NUM);

    while(nh.ok())
    {
        torque.data[0] = torque2current.current2torque(current.data[0]);
        target_current.data[0] = torque2current.torque2current(target_torque.data[0]);

        torque_pub.publish(torque);
        target_current_pub.publish(target_current);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}