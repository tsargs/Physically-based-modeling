//
//  ParticleGenerator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef PARTICLEGENERATOR
#define PARTICLEGENERATOR

#include <stdio.h>
#include <vector>
#include <stack>
#include <cmath>
#include <random>
#include <string>
#include "Particle.h"
#include "Matrix.h"
#include "Polygon.h"

class ParticleGenerator
{
    public:
        int rate;
        double fraction;
    
        std::vector<double> U;
        std::vector<double> N;
    
        int next_uniformly_distributed_number = 0;
        int next_normally_distributed_number = 0;
    
        const double INVERSE_RAND_MAX = 1/(double)RAND_MAX;
        const float TO_RADIANS_MULTIPLIER = 3.14159265359 / 180;
    
        Polygon* generator_polygon;
    
        bool is_streak = false;
    
        bool is_paper_plane = false;
    
        ParticleGenerator(void);
        ~ParticleGenerator(void);
    
        virtual void generate_particles(const double& h, std::vector<Particle>& particles, std::stack<int>& inactive_particles);
    
        virtual void update_generator_transform(const Vector3& position, const Vector3& normal);
    
        void set_particle_type(std::string type, std::string sub_type);
    
        void generate_uniformly_distributed_random_numbers(const int& count, const int& seed);
        void generate_normally_distributed_random_numbers(const int& count, const int& seed, const double& mean, const double& standard_deviation);
    
        double get_next_uniformly_distributed_random_number(void);
        double get_next_normally_distributed_random_number(void);
};

#endif
