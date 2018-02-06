//
//  Nozzle.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef NOZZLE
#define NOZZLE

#include <stdio.h>
#include "ParticleGenerator.h"

class Nozzle:public ParticleGenerator
{
    public:
        Vector3 c;
        float radius;
    
        Vector3 v;
        float angular_range;
    
        float particle_mass;
        float d;
    
        Vector3 start_color;
        Vector3 end_color;
    
        int life_span;
    
        Nozzle(void);
        Nozzle(const Vector3& center, const Vector3& velocity, const float& angle, const int& generation_rate, const float& mass, const float& surface_radius, const float& air_resistance, const int& particle_life_span, const Vector3& particle_start_color, const Vector3& particle_end_color);
        ~Nozzle(void);
    
        void generate_particles(const double& h, std::vector<Particle>& particles, std::stack<int>& inactive_particles);
        void update_generator_transform(const Vector3& position, const Vector3& normal);
        void create_polygon_circle(void);
};

#endif
