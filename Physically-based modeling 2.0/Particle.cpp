//
//  Particle.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Particle.h"

Particle::Particle(void)
{
    is_active = false;
    belongs_to_inactive_particles = false;
}

Particle::Particle(const Vector3& position, const Vector3& velocity, const float& life_span, const float& particle_mass, const float& air_resistance, const Vector3& start_col, const Vector3& end_col, const bool& status): pos(position), v(velocity), life_span(life_span), time_left(life_span), m(particle_mass), d(air_resistance), start_color(start_col), end_color(end_col), is_active(status), color(start_col), streak_start_pos(position)
{
}

Particle::~Particle(void)
{
    
}

bool Particle::update_status(void)
{
    if(is_active)
    {
        if(time_left <= 0)
        {
            is_active = false;
            belongs_to_inactive_particles = false;
        }
        else
        {
            float a = (time_left/life_span);
            color = start_color * a + end_color * (1-a);
        }
    }
    return is_active;
}

void Particle::update_color(void)
{
    if(time_left <= 0)
    {
        if(abs(time_left) >= life_span)
        {
            time_left = life_span;
        }
        else
        {
            float a = abs(time_left)/life_span;
            color = start_color * a + end_color * (1-a);
        }
    }
    else
    {
        float a = time_left/life_span;
        color = start_color * a + end_color * (1-a);
    }
}

void Particle::compute_banking_and_transformation_matrix(const float& k_banking, const float& banking_smoothing_constant)
{
    /*
    u_x = v.unit_vector();
    u_y = v.cross(a).unit_vector();
    u_z = u_x.cross(u_y);
     */
    
    ///*
    u_x = v.unit_vector();
    u_z = u_x.cross(Vector3(0,1,0));
    u_y = u_z.cross(u_x);
    //*/
    
    a_v = u_x * (a * u_x);
    a_t = a - a_v;
    
    float new_psi = atan(k_banking * (a_t * u_z));
    
    psi = (1-banking_smoothing_constant) * psi + banking_smoothing_constant * new_psi;
    
    t.set_translation(pos);
    r.add_vector3_as_column(u_x, 0);
    r.add_vector3_as_column(u_y, 1);
    r.add_vector3_as_column(u_z, 2);
    r.elements[3][3] = 1;
    r_x.set_rotation_x(psi);
    
    l_w = t*(r)*(r_x);
}

Vector3 Particle::transform_point(const Vector3& p)
{
    return l_w.multiply_with_point3d(p);
}
