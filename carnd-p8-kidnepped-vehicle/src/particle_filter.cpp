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
    if (yaw_rate == 0) {
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
  double min_dist = std::numeric_limits<double>::infinity();
  for (auto& observation : observations) {
    double x = observation.x;
    double y = observation.y;

    for (const auto& pred : predicted) {
      double distance = sqrt(pow(x - pred.x, 2) + pow(y - pred.y, 2));
      if (distance < min_dist) observation.id = pred.id;
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
  // transformation requires both rotation AND translation (but no scaling). The
  // following is a good resource for the theory:
  // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  // and the following is a good resource for the actual equation to implement
  // (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html

  double weight_normalization_term = 0.0;
  for (auto& particle : particles) {
    // Predicted measurement for each particles
    std::vector<LandmarkObs> predicted;

    double x = particle.x;
    double y = particle.y;
    double theta = particle.theta;

    // Get the predicted observations with the sensor range
    for (const auto& landmark : map_landmarks.landmark_list) {
      int id_i = landmark.id_i;
      double x_f = landmark.x_f;
      double y_f = landmark.y_f;
      if (sqrt(pow(x - x_f, 2) + pow(y - y_f, 2)) <= sensor_range) {
        LandmarkObs landmark_ob = {id_i, x_f, y_f};
        predicted.push_back(landmark_ob);
      }
    }

    // Transform observations from vehicle coordinate to map coordinate
    vector<LandmarkObs> observations_on_map;
    for (const auto& observation : observations) {
      LandmarkObs ob_on_map = {};
      ob_on_map.id = observation.id;
      ob_on_map.x =
          x + (cos(theta) * observation.x) - (sin(theta) * observation.y);
      ob_on_map.y =
          y + (sin(theta) * observation.y) + (cos(theta) * observation.y);
      observations_on_map.push_back(ob_on_map);
    }

    // Associate each predicted observations with observations
    dataAssociation(predicted, observations_on_map);

    // Set associations for debugging
    // vector<int> associations;
    // vector<double> sense_x;
    // vector<double> sense_y;
    // for (const auto& ob_on_map : observations_on_map) {
    //   associations.push_back(ob_on_map.id);
    //   sense_x.push_back(ob_on_map.x);
    //   sense_y.push_back(ob_on_map.y);
    // }
    // setAssociations(particle, associations, sense_x, sense_y);

    // Update weights with multivariate Gaussian
    double likelihood_measurements = 1.0;
    for (const auto& ob_on_map : observations_on_map) {
      int matched_id = ob_on_map.id;
      double mu_x, mu_y;
      for (auto& landmark : map_landmarks.landmark_list) {
        if (landmark.id_i == matched_id) mu_x = landmark.x_f;
        mu_y = landmark.y_f;
      }
      double sigma_x = std_landmark[0];
      double sigma_y = std_landmark[1];

      likelihood_measurements *=
          0.5 / (M_PI * sigma_x * sigma_y) *
          exp(-1.0 * (0.5 * pow(x - mu_x, 2) / pow(sigma_x, 2) +
                      0.5 * pow(y - mu_y, 2) / pow(sigma_y, 2)));
      cout << x << ", " << y << ", " << mu_x << ", " << mu_y << endl;
    }
    weight_normalization_term += likelihood_measurements;
    particle.weight = likelihood_measurements;
  }

  for (auto& particle : particles) {
    particle.weight /= weight_normalization_term;
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to
  // their weight. NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  vector<Particle> new_particles;

  vector<double> particle_weights;
  for (const auto& particle : particles) {
    particle_weights.push_back(particle.weight);
  }

  default_random_engine gen;
  std::discrete_distribution<int> discrete_generator(particle_weights.begin(),
                                                     particle_weights.end());

  for (int i = 0; i < particles.size(); i++) {
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
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

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
