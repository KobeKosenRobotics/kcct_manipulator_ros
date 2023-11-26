#ifndef PUBLISH_MANAGER_H
#define PUBLISH_MANAGER_H

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

namespace ec_calculator
{
    class PublishManager
    {
        private:
            int _joint_num = 1;
            std_msgs::Bool _emergency_stop;
            std_msgs::Bool _ik_enable;
            std_msgs::Bool _motor_enable;
            std_msgs::Bool _polygon_enable;
            std_msgs::Bool _simulation_enable;
            std_msgs::Bool _torque_enable;

            std_msgs::Float32MultiArray _target_angle;
            std_msgs::Float32MultiArray _target_pose;    // 2: start_joint, end_joint, 6: 3position, 3orientation

        public:
            PublishManager();

            void setJointNum(const int &joint_num_);
            void setEmergencyStop(const bool &emergency_stop_);
            void setIKEnable(const bool &ik_enable_);
            void setMotorEnable(const bool &motor_enable_);
            void setPolygonEnable(const bool &polygon_enable_);
            void setSimulationEnable(const bool &simulation_enable_);
            void setTorqueEnable(const bool &torque_enable_);
            template<class... A> void setTargetAngle(const A&... target_angle_)
            {
                int length = sizeof...(target_angle_);
                double target_angle_array_[] = {target_angle_...};

                for(int i = 0; i < length; i++)
                {
                    if(i >= _joint_num) return;
                    _target_angle.data[i] = target_angle_array_[i];
                }
            }
            void setTargetPose(const double &x_, const double &y_, const double &z_, const double &ez_, const double &ey_, const double &ex_, const double &scale_);

            std_msgs::Bool getEmergencyStop();
            std_msgs::Bool getIKEnable();
            std_msgs::Bool getMotorEnable();
            std_msgs::Bool getPolygonEnable();
            std_msgs::Bool getSimulationEnable();
            std_msgs::Bool getTorqueEnable();
            std_msgs::Float32MultiArray getTargetAngle();
            std_msgs::Float32MultiArray getTargetPose();
    };
}

#endif