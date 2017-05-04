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
#include <limits>
#include "particle_filter.h"
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Avoid multiple calls
	if(is_initialized)
		return;

  // How many particle do I need ? Not clear here how decide...
  // 10 are not enough but 11 are enough...should I use the minimum number??
	num_particles = 50;
	default_random_engine gen;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {
		Particle dummy;
		dummy.id = i;
    	dummy.x = dist_x(gen);
    	dummy.y = dist_y(gen);
    	dummy.theta = dist_psi(gen);
	  	dummy.weight = 1;
		particles.push_back(dummy);

    weights.push_back(1);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  	default_random_engine gen;
	normal_distribution<double> noise_x(0, std_pos[0]);
	normal_distribution<double> noise_y(0, std_pos[1]);
	normal_distribution<double> noise_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; ++i) {
		Particle &p = particles.at(i);
		if (abs(yaw_rate) > std::numeric_limits<float>::epsilon()) {
			double coeff1 = velocity / yaw_rate;
			double coeff2 = yaw_rate * delta_t;
			p.x += coeff1 * (sin(p.theta + coeff2) - sin(p.theta)) + noise_x(gen);
	    	p.y += coeff1 * (-cos(p.theta + coeff2) + cos(p.theta)) + noise_y(gen);
	    	p.theta += coeff2 + noise_theta(gen);
		} else {
  			p.x += velocity * delta_t * cos(p.theta) + noise_x(gen);
			p.y += velocity * delta_t * sin(p.theta) + noise_y(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	std::vector<LandmarkObs>::const_iterator it_p;
	std::vector<LandmarkObs>::iterator it_o;
	for(it_o = observations.begin(); it_o != observations.end(); ++it_o) {
  		LandmarkObs& obs = *it_o;
		int min_idx = 0;
		double min_dist = std::numeric_limits<double>::max();
		for(it_p = predicted.begin(); it_p != predicted.end(); ++it_p) {
			double tmp_dist = dist(it_o->x, it_o->y, it_p->x, it_p->y);
			if(tmp_dist < min_dist) {
				min_dist = tmp_dist;
				min_idx = it_p->id;
			}
		}
		// keep track of the assotiation by the ID! but remember that this id is
		// one bigger that the index that indicates the position of the associated data!!
		obs.id = min_idx;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	std::vector<LandmarkObs> observations, Map map_landmarks) {

	double prob;
	const double std_x = std_landmark[0];
	const double std_y = std_landmark[1];
	const double coeff = 1/(2 * M_PI * std_x * std_y);
  	for(int i=0; i < num_particles; ++i) {
    	Particle &p = particles.at(i);
		// Convert coordinates from car system to map coordinates
		std::vector<LandmarkObs> map_coord_observations;
        for(int j = 0; j < observations.size(); ++j) {
			LandmarkObs map_coord_observation;
			double x = observations.at(j).x;
			double y = observations.at(j).y;
			map_coord_observation.x = p.x + x * cos(p.theta) - y * sin(p.theta);
			map_coord_observation.y = p.y + x * sin(p.theta) + y * cos(p.theta);
			// we will use this id when associating map_coords with prediction
			map_coord_observation.id = -1;

			map_coord_observations.push_back(map_coord_observation);
        }
		// Extract element that are closer than sensor range from landmarks
		std::vector<LandmarkObs> predicted;
        for(int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
			const Map::single_landmark_s& lm = map_landmarks.landmark_list.at(j);
            double x = lm.x_f;
            double y = lm.y_f;
            int id = lm.id_i;
            double distance = dist(x, y, p.x, p.y);
            if(distance <= sensor_range) {
				LandmarkObs land_obs;
				land_obs.x = x;
  				land_obs.y = y;
  				land_obs.id = id;
				predicted.push_back(land_obs);
            }
		}

		// Now we have inside map_coord_observations in the id the index of the
		// closest landmak associated with the current observation...and we
		// modify the map coord entry associating the id of the corresponding
		// predicted element.
      	dataAssociation(predicted, map_coord_observations);

        // we can proceed in calculating the weights....
		prob = 1.0;
		for(int j=0; j < map_coord_observations.size(); ++j) {
			const LandmarkObs& obs = map_coord_observations.at(j);
			// remember that obs.id was 1 bigger than the value of the index!!!
			const Map::single_landmark_s& land = map_landmarks.landmark_list.at(obs.id-1);
			double x_term = pow((obs.x - land.x_f), 2) / (2 * pow(std_x, 2));
			double y_term = pow((obs.y - land.y_f), 2 ) / (2 * pow(std_y, 2));
            // Product of all terms
			prob *= coeff * exp(-(x_term + y_term));
		}
		p.weight = prob;
		weights[i] = prob;
  }
}

// Cleaner code here but I'd rather prefer trying to implement wheel algorighm
// anway both passes the tests :)))

// void ParticleFilter::resample() {
//   default_random_engine gen;
//   discrete_distribution<int> index_distrib(weights.begin(), weights.end());
//   std::vector<Particle> resampled;
//   for (int i = 0; i < num_particles; ++i)
//     resampled.push_back(particles.at(index_distrib(gen)));
//
//   particles = resampled;
// }

//Wheel algorithm
void ParticleFilter::resample() {
	default_random_engine gen;
	double max_weight = *max_element(weights.begin(), weights.end());
	uniform_real_distribution <double> beta_dist(0.0, 2 * max_weight);
  	double beta = 0.0;
	unsigned int index = rand() % num_particles;
	std::vector <Particle> resampled;

	for(int i = 0; i < num_particles; ++i) {
		beta = beta_dist(gen);
		while(beta > weights.at(index)) {
			beta -= weights.at(index);
			index = (index + 1) % num_particles;
		}
		resampled.push_back(particles.at(index));
	}
	particles = resampled;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
