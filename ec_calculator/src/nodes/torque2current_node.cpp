#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Core>

#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

const double deg2rad = M_PI/180.0, rad2deg = 180.0/M_PI, rpm2radps = 2*M_PI/60.0, radps2rpm = 60.0/(2*M_PI);
const int JOINT_NUM = 6;
class TorqueCurrentConverter
{
    private:
        double _a = 0.0;
        double _b = 0.0;
        double _c = 0.0;
        double _d = 0.0;
    public:
        void setMotorId(const int &motor_id_)
        {
            switch(motor_id_)
            {
                case 1:
                    _a = 4.006;
                    _b = 0.2176;
                    // _b = 0.30-0.02;
                    _c = 4.296;
                    _d = 0.1809;
                    // _d = 0.27-0.02;
                    break;
                case 2:
                    _a = 4.037;
                    _b = 0.1907;
                    _c = 4.880;
                    _d = 0.2619;
                    break;
                case 3:
                    _a = 3.362;
                    _b = 0.2199;
                    _c = 4.394;
                    _d = 0.3066;
                    break;
                case 4:
                    _a = 2.345;
                    // _b = 0.2591;
                    _b = 0.25;
                    _c = 3.441;
                    // _d = 0.2558;
                    _d = 0.23;
                    break;
                case 5:
                    _a = 2.356;
                    _b = 0.02650;
                    _c = 2.804;
                    _d = 0.0;
                    break;
                case 6:
                    _a = 2.943;
                    _b = 0.0+0.11;
                    _c = 3.728;
                    _d = 0.01448+0.11;
                    break;
                default:
                    break;
            }
        }
        double current2torque(const double &current_)
        {
            if(current_ < _b)
            {
                return _a*(current_+_b);
            }
            if(current_ > _d)
            {
                return _c*(current_-_d);
            }
            return 0.0;
        }
        double torque2current(const double &torque_)
        {
            if(torque_ < 0.0)
            {
                return (1/_a)*torque_-_b;
            }
            if(torque_ > 0.0)
            {
                return (1/_c)*torque_+_d;
            }
            return 0.0;
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

    TorqueCurrentConverter tcc[6];
    for(int i=0; i<6; i++)
    {
        tcc[i].setMotorId(i+1);
    }

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
        for(int i=0; i<6; i++)
        {
            torque.data[i] = tcc[i].current2torque(current.data[i]);
            target_current.data[i] = tcc[i].torque2current(target_torque.data[i]);
        }

        torque_pub.publish(torque);
        target_current_pub.publish(target_current);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}