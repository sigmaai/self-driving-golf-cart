//
// Created by neil on 4/9/19.
//

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){


    ros::init(argc, argv, "navigation_goal");
    ros::NodeHandle node;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 2 meters forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 0.2;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    ros::Rate rate(10.0);

    while (node.ok()){

        ROS_INFO("Sending goal");
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