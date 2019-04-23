//
// Created by neil on 4/10/19.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


tf::Quaternion toQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    tf::Quaternion q = tf::Quaternion(0, 0, 0, 0);
    q.setW(cy * cp * cr + sy * sp * sr);
    q.setX(cy * cp * sr - sy * sp * cr);
    q.setY(sy * cp * sr + cy * sp * cr);
    q.setZ(sy * cp * cr - cy * sp * sr);
    return q;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "localmap_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(60.0);
    while (node.ok()){
        transform.setRotation(toQuaternion(0, 0.0001, 0));
        transform.setOrigin( tf::Vector3(0.0, 0.0, -1.00) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "zed_camera_center", "base_link"));
        rate.sleep();
    }
    return 0;
}
