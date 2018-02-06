//
//  SPHNozzle.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SPHNozzle.h"


SPHNozzle::SPHNozzle(void)
{

}

SPHNozzle::SPHNozzle(const Vector3& center, const Vector3& velocity, const float& angle, const int& generation_rate, const float& mass, const float& surface_radius, const float& air_resistance, const Vector3& particle_color): c(center), radius(surface_radius), v(velocity), angular_range(angle), particle_mass(mass), d(air_resistance), color(particle_color)
{
    rate = generation_rate;
    generate_uniformly_distributed_random_numbers(500, 1);
    generate_normally_distributed_random_numbers(500, 2, 0.5, 0.16);
    create_polygon_circle();
}

SPHNozzle::~SPHNozzle(void)
{
    
}

void SPHNozzle::update_generator_transform(const Vector3& position, const Vector3& normal)
{
    c = position;
    v = normal;
    
    delete generator_polygon;
    create_polygon_circle();
}


void SPHNozzle::create_polygon_circle(void)
{
    generator_polygon = new Polygon();
    float theta = 0;
    float theta_increment_size = 30 * 3.14/180;
    
    Vector3 w = v.unit_vector();
    Vector3 u = (Vector3(0,1,0).cross(w)).unit_vector();
    Vector3 v = w.cross(u);
    
    while (theta < 2*3.14)
    {
        float x = radius * cos(theta);
        float y = radius * sin(theta);
        Vector3 p = c + u*x + v*y;
        generator_polygon->add_vertex(p.x, p.y, p.z);
        generator_polygon->color = color;
        
        theta += theta_increment_size;
    }
}

void SPHNozzle::generate_particles(const double& h, std::vector<SPHParticle>& particles, std::stack<int>& inactive_particles)
{
    int n = floor(rate * h);
    fraction += h * rate - n;
    
    if (fraction > 1)
    {
        ++n;
        --fraction;
    }
    
    for(int i = 0; i < n; i++)
    {
        if (!inactive_particles.empty())
        {
            int particle_index = inactive_particles.top();
            inactive_particles.pop();
            
            SPHParticle* particle = &particles[particle_index];
            
            Vector3 a(1, 0, 0);
            
            Vector3 u_z = v.unit_vector();
            Vector3 u_x = a.cross(u_z).unit_vector();
            Vector3 u_y = u_z.cross(u_x);
            
            Matrix m(3,3);
            
            m.add_vector3_as_column(u_x, 0);
            m.add_vector3_as_column(u_y, 1);
            m.add_vector3_as_column(u_z, 2);
            
            float f = get_next_normally_distributed_random_number();
            f = angular_range/3 * f + 0;
            float psi = sqrt(f) * angular_range * TO_RADIANS_MULTIPLIER;
            
            float theta = get_next_uniformly_distributed_random_number();
            theta = ((180 - (-180)) * theta + (-180)) * TO_RADIANS_MULTIPLIER;
            
            Vector3 v_offset_from_z_axis(cos(theta)*sin(psi), sin(theta)*sin(psi), cos(psi));
            
            float magnitude = get_next_uniformly_distributed_random_number();
            magnitude = (20 - 18) * magnitude + 2;
            
            f = get_next_uniformly_distributed_random_number();
            
            float r = sqrt(f) * radius;
            
            //here we are using the same theta calculated above since there is no difference in the way how they are calculated
            particle->pos = c + m.multiply(Vector3(r*cos(theta), r*sin(theta), 0));
            particle->streak_start_pos = particle->pos;
            particle->v = m.multiply(v_offset_from_z_axis)*magnitude;
            particle->m = particle_mass;
            particle->d = d;
            particle->is_streak = is_streak;
            particle->is_paper_plane = is_paper_plane;
            particle->color.x = color.x * rand()/RAND_MAX;
            particle->color.y = color.y * rand()/RAND_MAX;
            particle->color.z = color.z * rand()/RAND_MAX;
            particle->is_active = true;
            particle->is_resting = false;
            particle->belongs_to_inactive_particles = false;
        }
    }
}

void SPHNozzle::generate_particles_in_grid(std::vector<SPHParticle>& particles, std::stack<int>& inactive_particles)
{
    std::cout << "generate_particles_in_grid\n";
    
    int grid_dimension = 10;
    float grid_length = 20;
    
    float increment = (float)grid_length/(float)grid_dimension;
    
    Vector3 pos = Vector3(-60, -60, -60);
    Vector3 current_pos = pos;
    
    for (int i = 0; i < grid_dimension; i++)
    {
        for (int j = 0; j < grid_dimension; j++)
        {
            for (int k = 0; k < grid_dimension; k++)
            {
                if(inactive_particles.empty())
                    break;
                
                int particle_index = inactive_particles.top();
                inactive_particles.pop();
                
                SPHParticle* particle = &particles[particle_index];
                
                particle->pos = current_pos;
                particle->v = Vector3(0,0,0);
                particle->m = particle_mass;
                particle->d = d;
                particle->color.x = color.x * rand()/RAND_MAX;
                particle->color.y = color.y * rand()/RAND_MAX;
                particle->color.z = color.z * rand()/RAND_MAX;
                particle->is_active = true;
                particle->is_resting = false;
                particle->belongs_to_inactive_particles = false;
                
                current_pos.x += increment;
            }
            current_pos.x = pos.x;
            current_pos.y += increment;
        }
        current_pos.y = pos.y;
        current_pos.z += increment;
    }
}
