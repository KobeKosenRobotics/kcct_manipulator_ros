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

        geometry_msgs::TransformStamped testTF;
        testTF.header.frame_id = "base_link";
        testTF.child_frame_id = "test_link";
        testTF.header.stamp = ros::Time::now();
        testTF.transform.translation.x = 1.0f;
        testTF.transform.translation.y = 0.0f;
        testTF.transform.translation.z = 0.0f;
        testTF.transform.rotation.w = 1.0f;
        testTF.transform.rotation.x = 0.0f;
        testTF.transform.rotation.y = 0.0f;
        testTF.transform.rotation.z = 0.0f;

        br.sendTransform(testTF);
    }
}