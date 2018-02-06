//
//  SPHNozzle.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef SPHNOZZLE
#define SPHNOZZLE

#include <stdio.h>
#include "SPHParticleGenerator.h"

class SPHNozzle:public SPHParticleGenerator
{
    public:
        Vector3 c;
        float radius;
    
        Vector3 v;
        float angular_range;
    
        float particle_mass;
        float d;
    
        Vector3 color;
    
        SPHNozzle(void);
        SPHNozzle(const Vector3& center, const Vector3& velocity, const float& angle, const int& generation_rate, const float& mass, const float& surface_radius, const float& air_resistance, const Vector3& particle_color);
        ~SPHNozzle(void);
    
        void generate_particles(const double& h, std::vector<SPHParticle>& particles, std::stack<int>& inactive_particles);
        void update_generator_transform(const Vector3& position, const Vector3& normal);
        void create_polygon_circle(void);
        void generate_particles_in_grid(std::vector<SPHParticle>& particles, std::stack<int>& inactive_particles);
};

#endif
