#ifndef EC_CALCULATOR_MANIPULATOR_H

#include "model.h"
#include "joint.h"
#include "interpolation.h"
#include "differential_integral.h"

#include <chrono>
#include <string>
#include <fstream>
#include <sstream>

namespace ec_calculator
{
    class Manipulator
    {
        private:
            // Model
            Model* _model;
            int _JOINT_NUM, _CHAIN_NUM, _BINDING_CONDITIONS;
            Eigen::Matrix<double, -1, 6> _binding_conditions_matrix;
            std::vector<Joint> _joints;
            std::vector<int> _tip_index;   // joints[_tip_index[]] have no children.
            Eigen::Matrix<double, -1, -1> _angle_velocity_control_p_gain;
            double _pose_velocity_control_p_gain;
            double _angle_torque_control_p_gain;
            double _angle_torque_control_d_gain;
            double _gravitational_acceleration;
            Eigen::Matrix<double, 2, -1> _angle_limit;
            Eigen::Matrix<double, 2, -1> _angular_velocity_limit;
            Eigen::Matrix<double, 2, -1> _angular_acceleration_limit;
            double _jacobian_determinant_limit;

            // Parameters
            Eigen::Matrix<double, -1, 1> _angle;
            Eigen::Matrix<double, -1, 1> _target_angle;
            Eigen::Matrix<double, -1, 1> _angular_velocity;
            Eigen::Matrix<double, -1, 1> _target_angular_velocity;
            Eigen::Matrix<double, -1, 1> _angular_acceleration;
            Eigen::Matrix<double, -1, 1> _target_angular_acceleration;
            Eigen::Matrix<double, -1, 1> _torque;
            Eigen::Matrix<double, -1, 1> _target_torque;

            bool _emergency_stop = true;
            bool _ik_enable = false;
            bool _motor_enable = false;
            bool _polygon_enable = false;
            bool _simulation_enable = true;
            bool _torque_enable = false;

            // Inverse Kinematics
            int _ik_index = 0;
            std::vector<Interpolation> _ik_interpolation;
            std::vector<int> _start_joint_index;
            std::vector<int> _end_joint_index;
            Eigen::Matrix<double, -1, 1> _error_all;
            std::vector<Eigen::Matrix<double, 6, 1>> _target_pose;
            Eigen::Matrix<double, -1, -1> _jacobian;
                std::vector<Eigen::Matrix<double, 6, -1>> _jacobian_block;

            // Torque Control
            Interpolation _target_angle_interpolation;
            Eigen::Matrix<double, -1, -1> _Mf;
                std::vector<Eigen::Matrix<double, 6, -1>> _jacobian_g;
            Eigen::Matrix<double, -1, -1> _Cf;
                bool _was_dMab_dThc_calculated;
                std::vector<double> _dMab_dThc;
                bool _was_d_adj_inv_gab_d_thc_calculated;
                std::vector<Eigen::Matrix<double, 6, 6>> _d_adj_inv_gab_d_thc;
            Eigen::Matrix<double, -1, -1> _Nf;

            // Time
            std::chrono::system_clock::time_point _start_time, _end_time;
            double _during_time;
            bool _is_first_during_time_measurement = true; // TODO: Whether to reset when changing models

            std::chrono::system_clock::time_point _cumulative_time_start;
            double _cumulative_time = 0.0;
            bool _is_first_cumulative_time_measurement = true;

        public:
            // Initialize
            void init(Model* model_);
                void clearParameters();
                bool setChainMatrix(const Eigen::Matrix<bool, -1, -1> &chain_matrix_);
                void setBindingConditionsMatrix(const Eigen::Matrix<double, -1, 6> &binding_conditions_matrix_);
                void setJointParameters();
                    void setTipIndex(const int &tip_index_);
                    void clearTipIndex();
                void setAngleVelocityControlPGain(const Eigen::Matrix<double, -1, -1> &angle_velocity_control_p_gain_);
                void setPoseVelocityControlPGain(const double &pose_velocity_control_p_gain_);
                void setAngleTorqueControlPGain(const double &angle_torque_control_p_gain_);
                void setAngleTorqueControlDGain(const double &angle_torque_control_d_gain_);
                void setGravitationalAcceleration(const double &gravitational_acceleration_);
                void setAngleLimit(const Eigen::Matrix<double, 2, -1> &angle_limit_);
                void setAngularVelocityLimit(const Eigen::Matrix<double, 2, -1> &angular_velocity_limit_);
                void setAngularAccelerationLimit(const Eigen::Matrix<double, 2, -1> &angular_acceleration_limit_);
                void setJacobianDeterminantLimit(const double &jacobian_determinant_limit_);

            // Properties
            int getChainNum();
            int getJointNum();
            std::string getJointName(const int &joint_index_);
            std::string getJointParentName(const int &joint_index_);
            int getInverseKinematicsNum();
            bool getPolygonEnable();
            bool getSimulationEnable();
            bool getTorqueEnable();

            // Visualize
            double getVisualData(const int &joint_index_, const int &data_index_);

            // Forward Kinematics
            void updateAngle(const Eigen::Matrix<double, -1, 1> &angle_);
            void updateAngularVelocity(const Eigen::Matrix<double, -1, 1> &angular_velocity_);
            void updateAngularAcceleration(const Eigen::Matrix<double, -1, 1> &angular_acceleration_);
            void updateTorque(const Eigen::Matrix<double, -1, 1> &torque_);
            Eigen::Matrix<double, -1, 1> getAngle();
            Eigen::Matrix<double, 6, 1> getPose(const int &joint_index_);

            // AC or IK
            Eigen::Matrix<double, -1, 1> getAngularVelocity();

            // Angle Command
            Eigen::Matrix<double, -1, 1> getAngularVelocityByAngle(const Eigen::Matrix<double, -1, 1> &target_angle_);
            Eigen::Matrix<double, -1, 1> getAngularVelocityByAngle();

            // Inverse Kinematics
            void setJointPose(const int &start_joint_index_, const int &end_joint_index_, const Eigen::Matrix<double, 6, 1> &target_pose_);
                void clearJointPose();
            Eigen::Matrix<double, -1, 1> getAngularVelocityByEC();
                Eigen::Matrix<double, -1, -1> getJacobian();
                    Eigen::Matrix<double, 6, -1> getJacobianBlock(const int &ik_index_);

            // Torque Control
            Eigen::Matrix<double, -1, 1> getTorqueByAngle();
                Eigen::Matrix<double, -1, -1> getMf();
                    Eigen::Matrix<double, 6, -1> getJacobianG(const int &joint_index_);
                Eigen::Matrix<double, -1, -1> getCf();
                    double getCfBlock(const int &i_, const int &j_);
                        double get_dMab_dThc(const int &a_, const int &b_, const int &c_);
                            Eigen::Matrix<double, 6, 6> get_dAdjointInverseGab_dThc(const int &a_, const int &b_, const int &c_);
                Eigen::Matrix<double, -1, 1> getNf();

            // Angular Velocity to Angle (for Visualization)
            Eigen::Matrix<double, -1, 1> angularVelocity2Angle(const Eigen::Matrix<double, -1, 1> &angular_velocity_);
            // Torque to Angle (for Visualization)
            Eigen::Matrix<double, -1, 1> torque2Angle(const Eigen::Matrix<double, -1, 1> &torque_);

            // Subscriber
            void setEmergencyStop(const bool &emergency_stop_);
            void setIKEnable(const bool &ik_enable_);
            void setMotorEnable(const bool &motor_enable_);
            bool getMotorEnable();
            void setPolygonEnable(const bool &polygon_enable_);
            void setSimulationEnable(const bool &simulation_enable_);
            void setTorqueEnable(const bool &torque_enable_);
            void setTargetAngle(const Eigen::Matrix<double, -1, 1> &target_angle_);
            void setTargetPose(const Eigen::Matrix<double, -1, 1> &target_pose_);    // 2: start_joint, end_joint, 6: 3position, 3orientation
            void setTargetPolygon(const Eigen::Matrix<double, -1, 1> &target_polygon_);

            // Debug
            void printTree();
            void print();
            Eigen::Matrix<double, 6, 1> getTargetPose(const int &ik_index_);
            Eigen::Matrix<double, 6, 1> getMidPose(const int &ik_index_);
            void get_SCARA();
            Eigen::Matrix<double, 6, 1> getIdealTorque();
                DifferentialIntegral _angular_acc_diff;
                Eigen::Matrix<double, 6, 1> _ideal_torque;
            double updateCumulativeTime();
    };
}

#define EC_CALCULATOR_MANIPULATOR_H
#endif