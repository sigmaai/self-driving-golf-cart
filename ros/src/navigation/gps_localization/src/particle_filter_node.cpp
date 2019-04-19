//
// Created by Neil on 5/21/18.
// Particle filter node for localization
// (c) Yongyang Nie, 2018, All Rights Reserved

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Pose2D.h"
#include "particle_filter.h"
#include <math.h>
#include <iostream>
#include "json.hpp"

using namespace std;

// TODO: Remove once udaity code is removed
using json = nlohmann::json;

// this stores the current gps position estimate
geometry_msgs::Pose2D pos_est;
double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
int sensor_range = 50;


void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg){

    geometry_msgs::Pose2D p;
    p.x = msg->x;
    p.y = msg->y;
    p.theta = msg->theta;
    pos_est = p;
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
    ros::init(argc, argv, "particle_filter_node");

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
    ros::Subscriber pos_sub = n.subscribe("/localization/pose/map", 1000, pose_callback);
    ros::Rate loop_rate(30);

    /*
     * create a particle filter
     */
    ParticleFilter pf;
    if (!pf.initialized()){

        // Sense noisy position data from the simulator
        double est_x = pos_est.x;
        double est_y = pos_est.y;
        double est_theta = pos_est.theta;

        pf.init(est_x, est_y, est_theta, sigma_pos);
    }

    while (ros::ok()) {

        /**
         * This is a message object. You stuff it with data, and then publish it.
         */


        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */

        // TODO: adopt udacity code to C++ & ROS
        // TODO: 1. get measurement uncertainty
        // TODO: 2. keep track of previous velocity and yawrates
        // TODO: 3. get lidar observations
        // TODO: 4. somehow integrate map into this whole thing

        double delta_t = 0;
        double previous_velocity = 0;
        double previous_yawrate = 0;
        auto j = json::parse("");

        pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);


        // receive noisy observation data from the simulator
        // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
        vector<LandmarkObs> noisy_observations;
        string sense_observations_x = j[1]["sense_observations_x"];
        string sense_observations_y = j[1]["sense_observations_y"];

        std::vector<float> x_sense;
        std::istringstream iss_x(sense_observations_x);

        std::copy(std::istream_iterator<float>(iss_x),
                  std::istream_iterator<float>(),
                  std::back_inserter(x_sense));

        std::vector<float> y_sense;
        std::istringstream iss_y(sense_observations_y);

        std::copy(std::istream_iterator<float>(iss_y),
                  std::istream_iterator<float>(),
                  std::back_inserter(y_sense));

        for(int i = 0; i < x_sense.size(); i++)
        {
            LandmarkObs obs;
            obs.x = x_sense[i];
            obs.y = y_sense[i];
            noisy_observations.push_back(obs);
        }

        // Update the weights and resample
        // TODO: !
        // pf.updateWeights(sensor_range, std_landmark, noisy_observations, map);
        pf.resample();

        // Calculate and output the average weighted error of the particle filter over all time steps so far.
        vector<Particle> particles = pf.particles;
        int num_particles = particles.size();
        double highest_weight = -1.0;
        Particle best_particle;
        double weight_sum = 0.0;
        for (int i = 0; i < num_particles; ++i) {
            if (particles[i].weight > highest_weight) {
                highest_weight = particles[i].weight;
                best_particle = particles[i];
            }
            weight_sum += particles[i].weight;
        }
        cout << "highest w " << highest_weight << endl;
        cout << "average w " << weight_sum/num_particles << endl;


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}