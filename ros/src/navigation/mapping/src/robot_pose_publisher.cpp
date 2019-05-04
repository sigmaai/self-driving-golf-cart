/*!
 *
 * Created by Neil Nie. TF publisher for the
 * base_link frame and the laser_base frame.
 *
 * \author Neil Nie - contact@neilnie.com
 * \date May 3, 2019
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

bool first_msg_received = false;
geometry_msgs::PoseWithCovarianceStamped base_link_pose;

void pose_callback (const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    // set the global variable.
    base_link_pose = pose;
    first_msg_received = true;
}

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

    ros::init(argc, argv, "base_link_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform_base_link;
    tf::Transform transform_laser;

    ros::Subscriber sub = node.subscribe ("/rtabmap/localization_pose", 5, pose_callback);

    ros::Rate rate(50.0);

    while (node.ok()){

        if (first_msg_received){

            // base frame rotation and origin
            transform_base_link.setRotation(tf::Quaternion(base_link_pose.pose.pose.orientation.x,
                                                           base_link_pose.pose.pose.orientation.y,
                                                           base_link_pose.pose.pose.orientation.z,
                                                           base_link_pose.pose.pose.orientation.w));
            transform_base_link.setOrigin(tf::Vector3(
                    base_link_pose.pose.pose.position.x,
                    base_link_pose.pose.pose.position.y,
                    base_link_pose.pose.pose.position.z));

            // laser frame rotation and origin
            transform_laser.setRotation(toQuaternion(0.0, 0.0, 0.0));
            transform_laser.setOrigin(tf::Vector3(0, 0, 0.75));

        }else {

            // base frame rotation and origin
            transform_base_link.setRotation(toQuaternion(0.0, 0.0, 0.0));
            transform_base_link.setOrigin(tf::Vector3(0, 0, 0));

            // laser frame rotation and origin
            transform_laser.setRotation(toQuaternion(0.0, 0.0, 0.0));
            transform_laser.setOrigin(tf::Vector3(0, 0, 0.75));
        }

        br.sendTransform(tf::StampedTransform(transform_base_link, ros::Time::now(), "/map", "/base_link"));
        br.sendTransform(tf::StampedTransform(transform_laser, ros::Time::now(), "/base_link", "/laser_base"));

        ros::spinOnce();

        rate.sleep();
    }

    ros::spin();

    return 0;
}
