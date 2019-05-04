/*!
 *
 * Created by Neil Nie.
 *
 *
 * \author Neil Nie - contact@neilnie.com
 * \date May 3, 2019
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ros/ros.h"
#include <mapping/set_initial_pose.h>

// ====
ros::ServiceClient client;
bool first_msg_received = false;
geometry_msgs::PoseWithCovarianceStamped base_link_pose;

// ====
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


void pose_callback (const geometry_msgs::PoseWithCovarianceStamped& pose){

    ROS_INFO("got message");

    // set the global variable.
    base_link_pose = pose;

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

    if (client.call(srv)){
        ROS_INFO("Done");
        ros::shutdown();
    }
    else{
        ROS_ERROR("Failed to call service set initial pose");
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "set_vehicle_init_pos");

    ros::NodeHandle n;
    client = n.serviceClient<mapping::set_initial_pose>("/zed/set_initial_pose");

    ros::Subscriber sub = n.subscribe ("/rtabmap/localization_pose", 5, pose_callback);

    ros::spin();

    return 0;
}