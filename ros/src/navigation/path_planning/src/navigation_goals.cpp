//
// Created by neil on 4/9/19.
//

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int destination = -1;

void dest_callback (const std_msgs::Int32& dest) {
    destination = dest.data;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "navigation_goal");
    ros::NodeHandle node;

    ros::Subscriber sub_rtabmap = node.subscribe ("/dest_select", 5, dest_callback);

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

    ros::Rate rate(8.0);

    bool running;

    while (node.ok()){

        switch (destination) {
            case 0:
                ROS_INFO("Sending goal");
                ROS_INFO("Dest. #1 received");
                running = true;
                goal.target_pose.pose.position.x = 14;
                goal.target_pose.pose.position.y = 0.9;
                goal.target_pose.pose.orientation.x = 0;
                goal.target_pose.pose.orientation.y = 0;
                goal.target_pose.pose.orientation.z = 0.13744;
                goal.target_pose.pose.orientation.w = 1.0;

                break;
            case 1:
                ROS_INFO("Sending goal");
                ROS_INFO("Dest. #2 received");
                running = true;
                goal.target_pose.pose.position.x = 43;
                goal.target_pose.pose.position.y = 12.5;
                goal.target_pose.pose.orientation.x = 0;
                goal.target_pose.pose.orientation.y = 0;
                goal.target_pose.pose.orientation.z = 0.15325;
                goal.target_pose.pose.orientation.w = 0.988;

                break;
            case 2:
                ROS_INFO("Sending goal");
                ROS_INFO("Dest. #3 received");
                running = true;
                goal.target_pose.pose.position.x = 62;
                goal.target_pose.pose.position.y = 17;
                goal.target_pose.pose.orientation.x = 0;
                goal.target_pose.pose.orientation.y = 0;
                goal.target_pose.pose.orientation.z = 0.09449;
                goal.target_pose.pose.orientation.w = 1.0;

                break;
            default: // code to be executed if n doesn't match any cases
                ROS_WARN("Waiting...");
                running = false;
        }

        if (running) {

            ac.sendGoal(goal);
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray, the base is moving forward");
            else if (ac.getState() == actionlib::SimpleClientGoalState::REJECTED)
                ROS_ERROR("Action rejected");
            else if (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
                ROS_ERROR("Action pending");
            else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
                ROS_ERROR("Action lost");
        }

        rate.sleep();

        ros::spinOnce();
    }

    ros::spin();

    return 0;
}