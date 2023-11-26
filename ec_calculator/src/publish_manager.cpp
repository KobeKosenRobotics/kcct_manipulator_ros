#include "ec_calculator/publish_manager.h"

namespace ec_calculator
{
    PublishManager::PublishManager()
    {
        _emergency_stop.data = true;
        _ik_enable.data = false;
        _motor_enable.data = false;
        _polygon_enable.data = false;
        _simulation_enable.data = true;
        _torque_enable.data = false;
        _target_angle.data.resize(_joint_num);
        _target_pose.data.resize(7);
    }

    void PublishManager::setJointNum(const int &joint_num_)
    {
        _joint_num = joint_num_;
        _target_angle.data.resize(_joint_num);
    }

    void PublishManager::setEmergencyStop(const bool &emergency_stop_)
    {
        _emergency_stop.data = emergency_stop_;
    }

    void PublishManager::setIKEnable(const bool &ik_enable_)
    {
        _ik_enable.data = ik_enable_;
    }

    void PublishManager::setMotorEnable(const bool &motor_enable_)
    {
        _motor_enable.data = motor_enable_;
    }

    void PublishManager::setPolygonEnable(const bool &polygon_enable_)
    {
        _polygon_enable.data = polygon_enable_;

        if(_polygon_enable.data) _target_pose.data.resize(7);
        else _target_pose.data.resize(8);
    }

    void PublishManager::setSimulationEnable(const bool &simulation_enable_)
    {
        _simulation_enable.data = simulation_enable_;
    }

    void PublishManager::setTorqueEnable(const bool &torque_enable_)
    {
        _torque_enable.data = torque_enable_;
    }

    void PublishManager::setTargetPolygon(const double &x_, const double &y_, const double &z_, const double &ez_, const double &ey_, const double &ex_, const double &scale_)
    {
        setPolygonEnable(true);

        _target_pose.data[0] = x_;
        _target_pose.data[1] = y_;
        _target_pose.data[2] = z_;
        _target_pose.data[3] = ez_;
        _target_pose.data[4] = ey_;
        _target_pose.data[5] = ex_;
        _target_pose.data[6] = scale_;
    }

    void PublishManager::setTargetPose(const int &start_joint_, const int &end_joint_, const double &x_, const double &y_, const double &z_, const double &ez_, const double &ey_, const double &ex_)
    {
        setPolygonEnable(false);

        _target_pose.data[0] = start_joint_;
        _target_pose.data[1] = end_joint_;
        _target_pose.data[2] = x_;
        _target_pose.data[3] = y_;
        _target_pose.data[4] = z_;
        _target_pose.data[5] = ez_;
        _target_pose.data[6] = ey_;
        _target_pose.data[7] = ex_;
    }

    std_msgs::Bool PublishManager::getEmergencyStop()
    {
        return _emergency_stop;
    }

    std_msgs::Bool PublishManager::getIKEnable()
    {
        return _ik_enable;
    }

    std_msgs::Bool PublishManager::getMotorEnable()
    {
        return _motor_enable;
    }

    std_msgs::Bool PublishManager::getPolygonEnable()
    {
        return _polygon_enable;
    }

    std_msgs::Bool PublishManager::getSimulationEnable()
    {
        return _simulation_enable;
    }

    std_msgs::Bool PublishManager::getTorqueEnable()
    {
        return _torque_enable;
    }

    std_msgs::Float32MultiArray PublishManager::getTargetAngle()
    {
        return _target_angle;
    }

    std_msgs::Float32MultiArray PublishManager::getTargetPose()
    {
        return _target_pose;
    }
}