//
//  SPHParticleGenerator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SPHParticleGenerator.h"

SPHParticleGenerator::SPHParticleGenerator(void)
{
    //generate_samples();
}

SPHParticleGenerator::~SPHParticleGenerator(void)
{
    
}

void SPHParticleGenerator::generate_particles(const double& h, std::vector<SPHParticle>& particles, std::stack<int>& inactive_particles)
{
    
}

void SPHParticleGenerator::update_generator_transform(const Vector3& position, const Vector3& normal)
{
    
}

void SPHParticleGenerator::generate_particles_in_grid(std::vector<SPHParticle>& particles, std::stack<int>& inactive_particles)
{
    
}

void SPHParticleGenerator::set_particle_type(std::string type, std::string sub_type)
{    
    if(type.compare("streak") == 0)
        is_streak = true;
    else if(type.compare("polygon") == 0)
    {
        if(sub_type.compare("paper_plane") == 0)
            is_paper_plane = true;
    }
}

void SPHParticleGenerator::generate_uniformly_distributed_random_numbers(const int& count, const int& seed)
{
    srand(seed);
    
    for (int i = 0; i < count; i++)
    {
        U.push_back(rand()*INVERSE_RAND_MAX);
    }
}

void SPHParticleGenerator::generate_normally_distributed_random_numbers(const int& count, const int& seed, const double& mean, const double& standard_deviation)
{
    std::default_random_engine e(seed);
    
    std::normal_distribution<double> normal_distribution(mean, standard_deviation);
    
    while(N.size() < count)
    {
        double new_number = normal_distribution(e);
        if (new_number >= (- 3 * standard_deviation) && new_number <= (3 * standard_deviation))
        {
            N.push_back(new_number);
        }
    }
}

double SPHParticleGenerator::get_next_uniformly_distributed_random_number(void)
{
    double random_number = U[next_uniformly_distributed_number];
    
    if (next_uniformly_distributed_number < U.size()-1)
        ++next_uniformly_distributed_number;
    else
        next_uniformly_distributed_number = 0;
    
    return random_number;
}

double SPHParticleGenerator::get_next_normally_distributed_random_number(void)
{
    double random_number = N[next_normally_distributed_number];
    
    if (next_normally_distributed_number < N.size()-1)
        ++next_normally_distributed_number;
    else
        next_normally_distributed_number = 0;
    
    return random_number;
}
