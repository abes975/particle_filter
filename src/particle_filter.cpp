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
	num_particles = 100;
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
		int min_idx = 0;
		double min_dist = std::numeric_limits<double>::max();
		for(it_p = predicted.begin(); it_p != predicted.end(); ++it_p) {
			double tmp_dist = dist(it_o->x, it_o->y, it_p->x, it_p->y);
			if(tmp_dist < min_dist) {
				min_dist = tmp_dist;
				min_idx = it_p - predicted.begin();
			}
		}
		obs = predicted.at(min_idx);
	}

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

	for(int i = 0; i < particles.size() ; ++i) {
		Particle &p = particles.at(i);

	//x' =  cos(a) * x  - sin(a) * y  + px
  //y' =  sin(a) * x  + cos(a) * y  + py
	}
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
