#include "ec_calculator/manipulator_tf_publisher.h"

namespace ec_calculator
{
    ManipulatorTFPublisher::ManipulatorTFPublisher(Manipulator& manipulator)
    {
        _manipulator = &manipulator;
    }

    void ManipulatorTFPublisher::publish()
    {
        static tf::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_stamped;
        static tf2::Quaternion q;

        int num_of_times = _manipulator->getChainNum() + _manipulator->getJointNum();

        for(int joint = 0; joint < num_of_times; joint++)
        {
            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = _manipulator->getJointParentName(joint);
            transform_stamped.child_frame_id = _manipulator->getJointName(joint);
            transform_stamped.transform.translation.x = _manipulator->getVisualData(joint, 0);
            transform_stamped.transform.translation.y = _manipulator->getVisualData(joint, 1);
            transform_stamped.transform.translation.z = _manipulator->getVisualData(joint, 2);

            q.setRPY(_manipulator->getVisualData(joint, 3), _manipulator->getVisualData(joint, 4), _manipulator->getVisualData(joint, 5));
            transform_stamped.transform.rotation.x = q.x();
            transform_stamped.transform.rotation.y = q.y();
            transform_stamped.transform.rotation.z = q.z();
            transform_stamped.transform.rotation.w = q.w();

            br.sendTransform(transform_stamped);
        }
    }

    void ManipulatorTFPublisher::publish(const std::string &base_name_, const std::string &point_name_, const Eigen::Matrix<double, 6, 1> &pose_)
    {
        static tf::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_stamped;
        static tf2::Quaternion q;

        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = base_name_;
        transform_stamped.child_frame_id = point_name_;
        transform_stamped.transform.translation.x = pose_(0,0);
        transform_stamped.transform.translation.y = pose_(1,0);
        transform_stamped.transform.translation.z = pose_(2,0);

        q.setRPY(pose_(5,0), pose_(4,0), pose_(3,0));
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        br.sendTransform(transform_stamped);
    }
}