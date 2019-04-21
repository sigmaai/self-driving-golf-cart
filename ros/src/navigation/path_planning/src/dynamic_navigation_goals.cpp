//
// Created by neil on 4/9/19.
//

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

tf::StampedTransform transform;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){


    ros::init(argc, argv, "dynamic_navigation_goals");
    ros::NodeHandle node;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 2 meters forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ros::Rate rate(3.0);

    tf::TransformListener listener;

    while (node.ok()){

        ROS_INFO("Sending goal");

        try {
            listener.lookupTransform("map", "base_link", ros::Time(), transform);
        }catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }

        geometry_msgs::TransformStamped tf_msg;
        tf::transformStampedTFToMsg(transform, tf_msg);
        float trans_x = tf_msg.transform.translation.x;
        float trans_y = tf_msg.transform.translation.y;

        goal.target_pose.pose.position.x = trans_x + 8;
        goal.target_pose.pose.position.y = trans_y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf_msg.transform.rotation.w);

        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 2 meters forward");
        else
            ROS_INFO("The base failed to move forward 2 meters for some reason");
        rate.sleep();
    }

    return 0;
}