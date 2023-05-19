#include "ec_calculator/model.h"
#include "ec_calculator/manipulator.h"
#include "ec_calculator/manipulator_tf_publisher.h"
#include "ec_calculator/eigenUtility.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

using namespace ec_calculator;

Manipulator manip;
ManipulatorTFPublisher tfPublisher(manip);
Model model;

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
    double rate = 10.0;
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
    manip.print();
    angular_velocity.data.resize(manip.getJointNum());
    target_angle.data.resize(manip.getJointNum());

    int cha = 4, joi = 7;
    Eigen::Matrix<bool, 4, 7> cha_ma;
    cha_ma <<
    // 1  2  3  4  5  6  7  8  9 10
    1, 1, 1, 1, 0, 0, 0,
    1, 1, 1, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 3, 11> joi_po;
    joi_po <<
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 2, 3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 3, 7> tra;
    tra.setZero();
    Eigen::Matrix<double, 3, 7> rot;
    rot <<
    0, 0, 0, 1, 0, 0, 0,
    0, 1, 0, 0, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 1;
    Eigen::Matrix<double, 7, 7> a2a_ga;
    a2a_ga.setIdentity();
    a2a_ga *= 2.0;
    double ec_ga = 2.0;

    model.changeModel(cha, joi, cha_ma, joi_po, tra, rot, a2a_ga, ec_ga);

    manip.init(&model);
    manip.printTree();
    manip.print();
    angular_velocity.data.resize(manip.getJointNum());
    target_angle.data.resize(manip.getJointNum());

    while(nh.ok())
    {
        tfPublisher.publish();

        manip.angularVelocity2Angle(manip.getAngularVelocityByAngle());    // callBack(){manip.setAngle(msg);} while(){pub.publish(manip.getAngularVelocity());}

        angular_velocity.data = EigenUtility.matrix2Array(manip.getAngularVelocityByAngle());
        angular_velocity_pub.publish(angular_velocity);

        manip.print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}