//
// Created by Neil on 5/21/18.
// Particle filter node for localization
// (c) Yongyang Nie, 2018, All Rights Reserved

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "particle_filter.h"

// this stores the current gps position estimate
sensor_msgs::NavSatFix gps_est;
double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]

void gps_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){

}

// the main method when particle filter is called
int main(int argc, char **argv){

    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher pos_pub = n.advertise<std_msgs::String>("/localization/particle_filter/fix", 1000);
    ros::Subscriber gps_sub = n.subscribe("/sensor/gps/fix", 1000, gps_fix_callback);
    ros::Rate loop_rate(30);

    /*
     * create a particle filter
     */
    ParticleFilter pf;
    if (!pf.initialized(){

        // Sense noisy position data from the simulator
        double sense_x = gps_est.longitude;
        double sense_y = std::stod(j[1]["sense_y"].get<std::string>());
        double sense_theta = std::stod(j[1]["sense_theta"].get<std::string>());

        pf.init(sense_x, sense_y, sense_theta, sigma_pos);
    }

    while (ros::ok()) {

        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        sensor_msgs::NavSatFix msg;


        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;

}