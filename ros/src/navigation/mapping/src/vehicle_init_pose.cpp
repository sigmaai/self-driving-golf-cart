/*!
 * Initialize vehicle position with rtabmap.
 * This node also automatically set the vehicle position
 * when ZED position tracking has discrepancies.
 *
 * Created by Neil Nie.
 * (c) Copyright, All Rights Reserved.
 *
 * \author Neil Nie - contact@neilnie.com
 * \date May 3, 2019
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <mapping/set_initial_pose.h>

// =========
ros::ServiceClient client;
bool initial_pose_set = false;
geometry_msgs::PoseWithCovarianceStamped rtabmap_pose;
geometry_msgs::PoseStamped zed_pose;
// =========

inline const char * const BoolToString(bool b){

    return b ? "true" : "false";
}

void toEulerAngle(double x, double y, double z, double w, double &roll, double &pitch, double &yaw){

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

}

double calc_distance(double x1, double y1, double z1, double x2, double y2, double z2) {

    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2));
}

void pose_callback (const geometry_msgs::PoseWithCovarianceStamped& pose){

    ROS_INFO("got message");

    // set the global variable.
    rtabmap_pose = pose;

    mapping::set_initial_pose srv;
    srv.request.x = pose.pose.pose.position.x;
    srv.request.y = pose.pose.pose.position.y;
    srv.request.z = pose.pose.pose.position.z + 1.6764;

    double roll, pitch, yaw;
    toEulerAngle(pose.pose.pose.orientation.x,
                 pose.pose.pose.orientation.y,
                 pose.pose.pose.orientation.z,
                 pose.pose.pose.orientation.w,
                 roll, pitch, yaw);

    srv.request.R = roll;
    srv.request.P = 0;
    srv.request.Y = yaw;

    if (!initial_pose_set) {
        if (client.call(srv)){
            ROS_INFO("Done");
            initial_pose_set = true;
        }
        else
            ROS_ERROR("Failed to call service set initial pose");
    }
}

void zed_pose_callback (const geometry_msgs::PoseStamped& pose) {

    float rtabmap_covariance;
    for (auto i = 0; i < rtabmap_pose.pose.covariance.size(); i++)
        rtabmap_covariance = rtabmap_covariance + rtabmap_pose.pose.covariance[i];

    float distance = calc_distance(rtabmap_pose.pose.pose.position.x,
                                   rtabmap_pose.pose.pose.position.y,
                                   rtabmap_pose.pose.pose.position.z,
                                   pose.pose.position.x,
                                   pose.pose.position.x,
                                   pose.pose.position.x);

    if (rtabmap_covariance < 0.06 && distance > 0.50 && distance < 2.50) {

        mapping::set_initial_pose srv;
        srv.request.x = pose.pose.position.x;
        srv.request.y = pose.pose.position.y;
        srv.request.z = pose.pose.position.z + 1.6764;

        double roll, pitch, yaw;
        toEulerAngle(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,
                     roll, pitch, yaw);

        srv.request.R = roll;
        srv.request.P = 0;
        srv.request.Y = yaw;

        if (client.call(srv))
            ROS_INFO("Done");
        else
            ROS_ERROR("Failed to call service set initial pose");
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "set_vehicle_init_pos");

    ros::NodeHandle n;
    client = n.serviceClient<mapping::set_initial_pose>("/zed/set_initial_pose");

    ros::Subscriber sub_rtabmap = n.subscribe ("/rtabmap/localization_pose", 5, pose_callback);
//    ros::Subscriber sub_zed = n.subscribe("/zed/pose", 5, zed_pose_callback);

    ros::spin();

    return 0;
}