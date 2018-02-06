//
//  ParticleSimulator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/01/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "ParticleSimulator.h"


ParticleSimulator::ParticleSimulator(void)
{
    
}

ParticleSimulator::ParticleSimulator(const int& count):max_particles(count)
{
    for (int i = 0; i < max_particles; i++)
    {
        //inactive_particles.push(i);
        particles.push_back(Particle());
    }
    
    for(int i = 0; i < 10; i++)
    {
        trail_positions[i] = pos;
    }
}

void ParticleSimulator::update_generator_transform(const Vector3& position, const Vector3& normal)
{
    generators[0]->update_generator_transform(position, normal);
}

void ParticleSimulator::update_simulation(const double& displayStep, const bool& generate_particles)
{
    //std::cout << "ParticleSimulator::update_simulation\n";
    
    double t = 0;
        
    while (t < displayStep)
    {
        if(generate_particles)
        {
            for (int i = 0 ; i < generators.size(); i ++)
            {
                generators[i]->generate_particles(h, particles, inactive_particles);
            }
        }
        
        for (int i = 0 ; i < particles.size(); i++)
        {
            bool is_active = particles[i].update_status();
            
            if(is_active)
            {
                pos = particles[i].pos;
                v   = particles[i].v;
                a   = particles[i].a;
                d   = particles[i].d;
                m   = particles[i].m;
                
                pos_prev = pos;
                v_prev = v;
                
                update_acceleration();
                integrate();
                t_hit = h;
                process_collision();
                
                particles[i].pos = pos;
                particles[i].v = v;
                particles[i].a = a;
                
                particles[i].time_left -= h;
                
                if (pos.distance(particles[i].streak_start_pos) > 1)
                    particles[i].streak_start_pos =  pos + (particles[i].streak_start_pos - pos).unit_vector() * 4;
            }
            else
            {
                if(!particles[i].belongs_to_inactive_particles)
                {
                    inactive_particles.push(i);
                    particles[i].belongs_to_inactive_particles = true;
                }
            }
        }
        t += h;
    }
    //update_trail_positions();
}

void ParticleSimulator::update_acceleration(void)
{
    Vector3 v_wind;
    
    for (int i = 0 ; i < winds.size(); i ++)
        v_wind = v_wind + winds[i].get_wind_velocity(pos);
    
    a = g + (v_wind - v) * (d/m);
}

void ParticleSimulator::integrate(void)
{
    Vector3 v_op;
    
    v = v_prev + a * h;
    
    if(vortex != nullptr)
    {
        v_op = vortex->get_velocity(pos, (v_prev + v)/2);
        Vector3 temp = (v_prev + v)/2;
        //std::cout << "temp: ";
        //temp.print();
        //v_op.print();
        pos = pos_prev + v_op * h;
    }
    else
    {
        pos = pos_prev + ((v_prev + v)/2)*h;
    }
}

void ParticleSimulator::process_collision(void)
{
    bool collision = check_collision();
    if (collision)
    {
        update_collision_response();
    }
}

bool ParticleSimulator::check_collision(void)
{
    double t_hit_tol = 0.0002;//0.00008;
    bool collision = false;
    
    for (int i = 0; i < polygon_count; i++)
    {
        Vector3 p = polygons[i].vertices[0];
        Vector3 n = polygons[i].normal;
        
        //Check collision with the plane that has the polygon
        double t_hit_new = ((p - pos)*n)/(v*n);
        
        if (t_hit_new < t_hit && t_hit_new >= t_hit_tol)
        {
            //Find the collision point
            Vector3 pos_hit_new = pos + (v * t_hit_new);
            
            //Project vertices to 2D plane
            //Calculate 2D edge vectors for each projected vertex
            //Form 2X2 matrix for each vertex with edge vector and vector toward x_hit_new
            //Find the determinant of the matrix for each row
            bool same_sign = polygons[i].check_determinants(pos_hit_new);
            
            if (same_sign)
            {
                if (t_hit_new < t_hit)
                {
                    colliding_polygons.clear();
                    t_hit = t_hit_new;
                    colliding_polygons.push_back(&polygons[i]);
                }
                collision = true;
            }
        }
    }
    return collision;
}

void ParticleSimulator::update_collision_response(void)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        Vector3 p = colliding_polygons[i]->vertices[0];
        Vector3 n = colliding_polygons[i]->normal;
        double cr = colliding_polygons[i]->cr;
        double cf = colliding_polygons[i]->cf;
        
        double dist = (pos - p)*n;
        
        pos = pos - n * (1 + cr) * dist;
        Vector3 v_normal = n*(v*n);
        Vector3 v_tangential = v - v_normal;
        
        v = v_normal*(-cr) + v_tangential*(1-cf);
    }
    colliding_polygons.clear();
}

void ParticleSimulator::add_generator(std::string type, const Vector3& center, const Vector3& v, const float& angle, const int& rate, const float& m, const float& surface_radius, const float& air_resistance, const int& life_span, const Vector3& start_color, const Vector3& end_color)
{
    if (type.compare("nozzle") == 0)
    {
        ParticleGenerator* generator = new Nozzle(center, v, angle, rate, m, surface_radius, air_resistance, life_span, start_color, end_color);
        generators.push_back(generator);
    }
}

ParticleSimulator::~ParticleSimulator()
{

}
