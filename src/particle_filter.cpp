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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // Avoid multiple calls
	if(is_initialized)
		return;

  // How many particle do I need ? Not clear here how decide...
	num_particles = 1000;
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
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;
	normal_distribution<double> noise_x(0, std_pos[0]);
	normal_distribution<double> noise_y(0, std_pos[1]);
	normal_distribution<double> noise_theta(0, std_pos[2]);


  double coeff1 = velocity / yaw_rate;
  double coeff2 = yaw_rate * delta_t;

	for (int i = 0; i < num_particles; ++i) {
		//if(i < 4)
		//	std::cout << i << ": " << particles.at(i).x << "\t" << particles.at(i).y << "\t" << particles.at(i).theta << std::endl;
/////////if (abs(yaw_rate) > 0.00001) {
		Particle &p = particles.at(i);
		p.x += coeff1 * (sin(p.theta + coeff2) - sin(p.theta)) + noise_x(gen);
    p.y += coeff1 * (-cos(p.theta + coeff2) + cos(p.theta)) + noise_y(gen);
    p.theta += coeff2 + noise_theta(gen);
	}
	//std::cout << "dopo update vediamo un po'" << std::endl;
	//for (int i = 0; i < 4; ++i)
	//	std::cout << i << ": " << particles.at(i).x << "\t" << particles.at(i).y << "\t" << particles.at(i).theta << std::endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	std::vector<LandmarkObs>::const_iterator it_p;
	std::vector<LandmarkObs>::iterator it_o;
	for(it_o = observations.begin(); it_o != observations.end(); ++it_o) {
		LandmarkObs& obs = *it_o;
		//cout << " Observation (prediction) x = " << obs.x << " y = " << obs.y << " id = " << obs.id << " size = " << observations.size() << endl;
		int min_idx = 0;
		double min_dist = std::numeric_limits<double>::max();
		for(it_p = predicted.begin(); it_p != predicted.end(); ++it_p) {
			double tmp_dist = dist(it_o->x, it_o->y, it_p->x, it_p->y);
			//cout << "\tPredicted (landmark) x = " << it_p->x << " y = " << it_p->y << " id = " << it_p->id <<  " distance  = " << tmp_dist << endl;
			if(tmp_dist < min_dist) {
				min_dist = tmp_dist;
				min_idx = it_p - predicted.begin();
				//cout << "\t\twe have a new min distance " << min_dist << " at element " << min_idx << endl;
			}
		}
		obs.id = min_idx;
	}
	//cout << "Print the associated LANDMARKS" << endl;
	for(it_o = observations.begin(); it_o != observations.end(); ++it_o) {
			LandmarkObs& aaa = *it_o;
			//cout << " Observation modified (prediction) x = " << aaa.x << " y = " << aaa.y << " id = " << aaa.id << endl;
	}
  //cout << "FINISHED ASSOCIATION LANDMARKS" << endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
  double weights_sum = 0;

	// std::vector<LandmarkObs> landmarks;
	// for(int i = 0; i < map_landmarks.landmark_list.size(); ++i) {
	// 		double x_lm = map_landmarks.landmark_list.at(i).x_f;
	// 		double y_lm = map_landmarks.landmark_list.at(i).y_f;
	// 		int id_lm = map_landmarks.landmark_list.at(i).id_i;
	// 		LandmarkObs land_obs;
 // 			land_obs.x = x_lm;
 // 			land_obs.y = y_lm;
 // 			land_obs.id = id_lm;
	// 		landmarks.push_back(land_obs);
  // }

  for(int i=0; i < num_particles; ++i){
        Particle &p = particles.at(i);
        std::vector<LandmarkObs> predicted;
        //std::map<int,int> lm2idx;
				// Convert coordinates from car system to map coordinates
        for(int j = 0; j < observations.size(); ++j){
            double x = observations.at(j).x;
            double y = observations.at(j).y;
						//x' =  cos(a) * x  - sin(a) * y  + px
						//y' =  sin(a) * x  + cos(a) * y  + py
            observations.at(j).x = x * cos(p.theta) - y * sin(p.theta) + p.x ;
            observations.at(j).y = x * sin(p.theta) + y * cos(p.theta) + p.y ;
        }

        for(int j = 0; j < map_landmarks.landmark_list.size(); ++j){
						const Map::single_landmark_s& lm = map_landmarks.landmark_list.at(j);
            double lm_x = lm.x_f;
            double lm_y = lm.y_f;
            int lm_id = lm.id_i;
            double distance = dist(lm_x, lm_y, p.x, p.y);
            if(distance <= sensor_range) {
									LandmarkObs land_obs;
			 						land_obs.x = lm_x;
				  				land_obs.y = lm_y;
				  				land_obs.id = lm_id;
          				predicted.push_back(land_obs);
            }
        }

				// Now we have inside observations in the id the index of the
				// closest landmak associated with the current observation...
      	dataAssociation(predicted, observations);

        // we can proceed in calculating the weights....
				double prob = 1.0;
				double std_x = std_landmark[0];
				double std_y = std_landmark[1];
				double coeff = 1/(2 * M_PI * std_x * std_y);
				for(int j=0; j < observations.size(); ++j) {
						const LandmarkObs& obs = observations.at(j);
						const Map::single_landmark_s& land = map_landmarks.landmark_list.at(obs.id);
						//cout << "Particle " << i << " sample: " << j << ": from particle x = " << obs.x << " y = " << obs.y << " id = " << obs.id << endl;
						//cout << "\t Landmark x = " << land.x << " y = " << land.y << "id = " <<  land.id << endl;
						double x_term = pow((obs.x - land.x_f), 2) / (2 * pow(std_x, 2));
					  double y_term = pow((obs.y - land.y_f),2 ) / (2 * pow(std_y, 2));
            // Product of all terms
					  prob *= coeff * exp(-(x_term + y_term));
				}
				cout << "Particle " << i << ": total weight is " << prob << endl;
			  p.weight = prob;
				weights[i] = prob;
				weights_sum += prob;
  }
	// Normalize weights in order to have a prob distribution
	for(int i = 0; i < weights.size(); ++i)
		weights[i] /= weights_sum;
}

// Cleaner code here but I'd rather prefer trying to implement wheel algorighm
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;
  discrete_distribution<int> index_distrib(weights.begin(), weights.end());
  std::vector<Particle> resampled;
  for (int i = 0; i < num_particles; ++i)
    resampled.push_back(particles[index_distrib(gen)]);

  particles = resampled;
}

// Wheel algorithm
// void ParticleFilter::resample() {
//   default_random_engine gen;
// 	double max_weight = *max_element(weights.begin(), weights.end());
// 	uniform_real_distribution <double> beta_dist(0.0, 2 * max_weight);
//   double beta = 0.0;
// 	unsigned int index = rand() % num_particles;
// 	std::vector <Particle> resampled;
//
// 	for(int i = 0; i < num_particles; ++i) {
// 		beta = beta_dist(gen);
// 		while(beta > weights.at(index)) {
// 			beta -= weights.at(index);
// 			index = (index + 1) % num_particles;
// 		}
// 		resampled.push_back(particles.at(index));
//   }
// 	particles = resampled;
// }

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
