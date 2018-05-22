/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Modified & completed by Neil Nie
 *  (C) Yongyang Nie 2018, All Rights Reserved
 *  Contact: contact@neilnie.com
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath>
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
            x_pred = pre_x + velocity / yaw_rate * (sin(theta_pred) - sin(pre_theta));
            y_pred = pre_y + velocity / yaw_rate * (cos(pre_theta) - cos(theta_pred));

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
        // list all landmarks within sensor range
        vector<LandmarkObs> predicted_landmarks;

        for (const auto& map_landmark : map_landmarks.landmark_list){

            double d = dist(current_x, current_y, map_landmark.x_f, map_landmark.y_f);

            if (d < sensor_range){

                LandmarkObs l_pred;
                l_pred.id = map_landmark.id_i;
                l_pred.x = map_landmark.x_f;
                l_pred.y = map_landmark.y_f;

                predicted_landmarks.push_back(l_pred);
            }
        }

        // List all observations in map coordinates
        vector<LandmarkObs> observed_landmarks_map_ref;
        for (int j = 0; j < observations.size(); j++){

            // convert observations from particle to map coordinates
            LandmarkObs translated_obs;
            translated_obs.x = cos(current_theta) * observations[j].x - sin(current_theta) * observations[j].y + current_x;
            translated_obs.y = sin(current_theta) * observations[j].x + cos(current_theta) * observations[j].y + current_y;

            observed_landmarks_map_ref.push_back(translated_obs);
        }

        dataAssociation(predicted_landmarks, observed_landmarks_map_ref);

        // Compute the likelihood for each particle, which is the probability of obtaining
        // current observations being in state (particle_x, particle_y, particle_theta)
        double particle_likelihood = 1.0;
        double mu_x, mu_y;
        for (const auto& obs: observed_landmarks_map_ref){

            // Find corresponding landmarks on map for centering gaussian distribution
            for (const auto& land: predicted_landmarks){

                if (obs.id == land.id) {
                    mu_x = land.x;
                    mu_y = land.y;
                    break;
                }
            }

            double norm_fac = 2 * M_PI * std_x * std_y;
            double prob = exp(-( pow(obs.x - mu_x, 2) / (2 * std_x * std_x) + pow(obs.y - mu_y, 2) / (2 * std_y * std_y)));

            particle_likelihood *= prob / norm_fac;
        }

        particles[i].weight = particle_likelihood;

    } // end of loop for each particle'

    double norm_fac = 0.0;
    for (const auto& particle: particles)
        norm_fac += particle.weight;

    // Normalize the weights so sum to one
    for (auto& particle: particles)
        particle.weight /= (norm_fac + numeric_limits<double>::epsilon());

}

void ParticleFilter::resample() {

	// std::discrete_distribution is helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    vector<double> particle_weights;
    for (const auto& particle: particles)
        particle_weights.push_back(particle.weight);

    discrete_distribution<int> weighted_distribution(particle_weights.begin(), particle_weights.end());

    vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++){
        int k = weighted_distribution(gen);
        resampled_particles.push_back(particles[k]);
    }

    particles = resampled_particles;

    // Reset weights for all particles
    for (auto& particle: particles)
        particle.weight = 1.0;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y) {

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
string ParticleFilter::getSenseX(Particle best) {

	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best) {

	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
