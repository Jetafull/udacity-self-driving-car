/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first
  // position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and
  // others in this file).

  // Add random Gaussian noise
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize particles with random generators
  for (int i = 0; i < num_particles; i++) {
    Particle particle = {};
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    weights.push_back(1.0);

    particles.push_back(particle);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and
  // std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;

  for (int i = 0; i < num_particles; i++) {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    // Apply motion model
    if (fabs(yaw_rate) < 0.01) {
      x += velocity * delta_t * cos(theta);
      y += velocity * delta_t * sin(theta);
    } else {
      x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      theta += yaw_rate * delta_t;
    }

    // Add random Gaussian noise
    normal_distribution<double> dist_x(x, std_pos[0]);
    normal_distribution<double> dist_y(y, std_pos[1]);
    normal_distribution<double> dist_theta(theta, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed
  // measurement and assign the observed measurement to this particular
  // landmark. NOTE: this method will NOT be called by the grading code. But you
  // will probably find it useful to implement this method and use it as a
  // helper during the updateWeights phase.
  for (auto& observation : observations) {
    double min_dist = std::numeric_limits<double>::infinity();
    double x = observation.x;
    double y = observation.y;

    for (const auto& pred : predicted) {
      double distance = calc_dist(x, y, pred.x, pred.y);
      if (distance < min_dist) {
        observation.id = pred.id;
        min_dist = distance;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs>& observations,
                                   const Map& map_landmarks) {
  // TODO: Update the weights of each particle using a mult-variate Gaussian
  // distribution. You can read more about this distribution here:
  //   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your
  // particles are located according to the MAP'S coordinate system. You will
  // need to transform between the two systems. Keep in mind that this
  // transformation requires both rotation AND translation (but no scaling).
  // The following is a good resource for the theory:
  // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  // and the following is a good resource for the actual equation to implement
  // (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html

  // normalization_term is the sum all particles' likelihood
  double weight_normalization_term = 0.0;
  double sigma_x = std_landmark[0];
  double sigma_y = std_landmark[1];

  for (auto& particle : particles) {
    // Predicted measurement for each particles
    std::vector<LandmarkObs> predicted;

    // Get the predicted observations with the sensor range
    for (const auto& landmark : map_landmarks.landmark_list) {
      double distance =
          calc_dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
      if (distance <= sensor_range) {
        LandmarkObs landmark_ob = {landmark.id_i, landmark.x_f, landmark.y_f};
        predicted.push_back(landmark_ob);
      }
    }

    // Transform observations from vehicle coordinate to map coordinate
    vector<LandmarkObs> observations_on_map;
    for (const auto& observation : observations) {
      LandmarkObs ob_on_map = {};
      ob_on_map.id = observation.id;
      ob_on_map.x = particle.x + cos(particle.theta) * observation.x -
                    sin(particle.theta) * observation.y;
      ob_on_map.y = particle.y + sin(particle.theta) * observation.x +
                    (cos(particle.theta) * observation.y);
      observations_on_map.push_back(ob_on_map);
    }

    dataAssociation(predicted, observations_on_map);

    // Update weights with multivariate Gaussian
    double likelihood_measurements = 1.0;

    // Use x and y from nearest landmark
    for (const auto& ob_on_map : observations_on_map) {
      // The ob_on_map.id is the Map Id, not list index. It starts from 1.
      double mu_x = map_landmarks.landmark_list[ob_on_map.id - 1].x_f;
      double mu_y = map_landmarks.landmark_list[ob_on_map.id - 1].y_f;

      double prob = calc_gaussian_prob_2d(ob_on_map.x, ob_on_map.y, mu_x, mu_y,
                                          sigma_x, sigma_y);
      likelihood_measurements *= prob;
    }

    particle.weight = likelihood_measurements;
    weight_normalization_term += likelihood_measurements;
  }

  for (int i = 0; i < num_particles; i++) {
    particles[i].weight /= weight_normalization_term;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional
  // to their weight. NOTE: You may find std::discrete_distribution helpful
  // here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  vector<Particle> new_particles;

  default_random_engine gen;
  discrete_distribution<int> discrete_generator(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; i++) {
    int sampled_id = discrete_generator(gen);
    new_particles.push_back(particles[sampled_id]);
  }

  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle,
                                         const std::vector<int>& associations,
                                         const std::vector<double>& sense_x,
                                         const std::vector<double>& sense_y) {
  // particle: the particle t(geo assign each listed association, and
  // association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed
  // association sense_x: the associations x mapping already converted to
  // world coordinates sense_y: the associations y mapping already converted
  // to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
