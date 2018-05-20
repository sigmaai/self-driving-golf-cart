/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;

// Particle filter constructor
// Set the number of particles and initialize the positions based on GPS estimate
void ParticleFilter::init(double x, double y, double theta, double std[]) {

	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 100;

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++){

        Particle p;
        p.id = i;
        p.weight = 1.0;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);

        particles.push_back(p);

    }

    is_initialized = true;

}


// move each particle according to the bicycle motion model, with added noise
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    for (int i = 0; i < num_particles; i++){

        double pre_x = particles[i].x;
        double pre_y = particles[i].y;
        double pre_theta = particles[i].theta;

        double x_pred, y_pred, theta_pred;

        if (abs(yaw_rate) > 1e-5){

            theta_pred = pre_theta + yaw_rate * delta_t;
            x_pred = pre_x + velocity / yaw_rate * (sin(x_pred) - sin(pre_x));
            y_pred = pre_y + velocity / yaw_rate * (cos(pre_y) - cos(y_pred));

        }else{
            theta_pred = pre_theta;
            x_pred = pre_x + velocity * delta_t * cos(pre_theta);
            y_pred = pre_y + velocity * delta_t * sin(pre_theta);

        }

        normal_distribution<double> dist_x(x_pred, std_pos[0]);
        normal_distribution<double> dist_y(y_pred, std_pos[1]);
        normal_distribution<double> dist_theta(theta_pred, std_pos[2]);

        // update particle with noise
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);

    }

}

// Find which observations correspond to with landmarks by using nearest neighbor.
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	//   observed measurement to this particular landmark.
	//   implement this method and use it as a helper during the updateWeights phase.

    for (auto& obs: observations){

        double min_dist = numeric_limits<double>::max();

        for (const auto& pred_obs: predicted){

            double d = dist(obs.x, obs.y, pred_obs.x, pred_obs.y);
            if (d < min_dist){
                obs.id = pred_obs.id;
                min_dist = d;
            }
        }
    }

}

// update the weight of each particle taking into account current measurements
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    double std_x = std_landmark[0];
    double std_y = std_landmark[1];

    for (int i = 0; i < num_particles; i++){

        double current_x = particles[i].x;
        double current_y = particles[i].y;
        double current_theta = particles[i].theta;

        // LandMarkObs are in vehicle coordinates
        vector<LandmarkObs> predicted_landmarks;

        for (const auto& map_landmark : map_landmarks.landmark_list){

            int l_id = map_landmark.id_i;
            double l_x = (double) map_landmark.x_f;
            double l_y = (double) map_landmark.y_f;

            double d = dist(current_x, current_y, l_x, l_y);

            // check if distance from vehicle to landmark is
            // less than sensor range
            if (d < sensor_range){

                LandmarkObs l_pred;
                l_pred.id = l_id;
                l_pred.x = l_x;
                l_pred.y = l_y;

                predicted_landmarks.push_back(l_pred);
            }
        }
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
