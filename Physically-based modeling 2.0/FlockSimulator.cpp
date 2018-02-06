//
//  FlockSimulator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/01/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "FlockSimulator.h"


FlockSimulator::FlockSimulator(void)
{
    
}

FlockSimulator::FlockSimulator(const int& count):max_particles(count)
{
    for (int i = 0; i < max_particles; i++)
    {
        particles.push_back(Particle());
        particles[i].belongs_to_inactive_particles = true;
        inactive_particles.push(max_particles-1-i);
    }
    
    for(int i = 0; i < 10; i++)
    {
        trail_positions[i] = pos;
    }
}

void FlockSimulator::update_generator_transform(const Vector3& position, const Vector3& normal)
{
    generators[0]->update_generator_transform(position, normal);
}

void FlockSimulator::update_simulation(const double& displayStep, const bool& generate_particles)
{
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
        
        update_lead_boid_simulation();
        
        for (int i = 1 ; i < particles.size(); i++)
        {
            if (particles[i].is_active)
            {
                pos_i = particles[i].pos;
                v_i = particles[i].v;
                
                a_a = ZERO_VECTOR;
                a_v = ZERO_VECTOR;
                a_c = ZERO_VECTOR;
                
                bool boid_interaction_calculated = false;
                
                for(int j = 0; j < particles.size(); j++)
                {
                    if(i != j && particles[j].is_active)
                    {
                        pos_j = particles[j].pos;
                        v_j = particles[j].v;
                        
                        vector_i_j = pos_j - pos_i;
                        
                        if(j == 0)
                        {
                            kd = r1;
                            ktheta = theta1;
                        }
                        else
                        {
                            kd = pos_i.distance(pos_j);
                            ktheta = abs(v_i.angle(vector_i_j));
                        }
                        
                        if(kd < r2 && ktheta < theta2)
                        {
                            d_i_j = pos_i.distance(pos_j);
                            
                            a_a_i_j = vector_i_j.unit_vector() * (-ka/d_i_j);
                            a_v_i_j = (v_j - v_i) * kv;
                            a_c_i_j = vector_i_j * kc;
                            
                            if(j == 0)
                            {
                                kd = 1;
                                ktheta = 1;
                            }
                            else
                            {
                                //calculate distance factor
                                if(kd <= r1)
                                    kd = 1;
                                else
                                    kd = (r2-kd)/(r2-r1);
                                
                                //calculate angle factor
                                if(ktheta <= theta1)
                                    ktheta = 1;
                                else
                                    ktheta = (theta2-ktheta)/(theta2-theta1);
                            }
                            
                            a_a = a_a + a_a_i_j * kd * ktheta;
                            a_v = a_v + a_v_i_j * kd * ktheta;
                            a_c = a_c + a_c_i_j * kd * ktheta;
                            
                            boid_interaction_calculated = true;
                        }
                    }
                }
                
                pos = pos_i;
                v   = v_i;
                d   = particles[i].d;
                m   = particles[i].m;
                
                pos_prev = pos;
                v_prev = v;
                
                if(boid_interaction_calculated)
                {
                    a_r = a_max;
                    
                    a = a_a.unit_vector() * std::min(a_r, (float)a_a.magnitude());
                    a_r = a_max - a.magnitude();
                    
                    a = a + a_v.unit_vector() * std::min(a_r, (float)a_v.magnitude());
                    a_r = a_max - a.magnitude();
                    
                    a = a + a_c.unit_vector() * std::min(a_r, (float)a_c.magnitude());
                }
                else
                    update_acceleration();
                
                for(int k = 0; k < obstacles.size(); k++)
                    a = a + obstacles[k]->get_steering_acceleration(pos, v, a);
                
                integrate_boid();
                
                particles[i].update_color();
                particles[i].pos = pos;
                particles[i].v = v;
                particles[i].a = a;
                particles[i].time_left -= h;
                
                if(particles[i].is_streak)
                {
                    if (pos.distance(particles[i].streak_start_pos) > 1)
                        particles[i].streak_start_pos =  pos + (particles[i].streak_start_pos - pos).unit_vector() * 4;
                }
                else if (particles[i].is_paper_plane)
                {
                    particles[i].compute_banking_and_transformation_matrix(k_banking, banking_smoothing_constant);
                }
            }
        }
        t += h;
    }
}

void FlockSimulator::update_lead_boid_simulation(void)
{
    pos = particles[0].pos;
    v   = particles[0].v;
    a   = particles[0].a;
    d   = particles[0].d;
    m   = particles[0].m;
    
    pos_prev = pos;
    v_prev = v;
    
    if(lead_boid_controlled_by_physics)
    {
        update_acceleration();
        integrate();
        t_hit = h;
        process_collision();
        
        particles[0].pos = pos;
        particles[0].v = v;
        particles[0].a = a;
        
        particles[0].is_streak = true;
        
        if (pos.distance(particles[0].streak_start_pos) > 1)
            particles[0].streak_start_pos =  pos + (particles[0].streak_start_pos - pos).unit_vector() * 4;
    }
    else if(lead_boid_controlled_by_sine_function)
    {
        a = Vector3(1.5/lead_boid_speed, sin(pos.y*0.2), sin(pos.y*0.2)) * lead_boid_speed - v * (d/m);
        
        integrate();
        
        particles[0].pos = pos;
        particles[0].v = v;
        particles[0].a = a;
        
        particles[0].is_streak = true;
        
        if (pos.distance(particles[0].streak_start_pos) > 1)
            particles[0].streak_start_pos =  pos + (particles[0].streak_start_pos - pos).unit_vector() * 4;
    }
}

void FlockSimulator::update_acceleration(void)
{
    Vector3 v_wind;
    
    for (int i = 0 ; i < winds.size(); i ++)
        v_wind = v_wind + winds[i].get_wind_velocity(pos);
    
    a = g + (v_wind - v) * (d/m);
}

void FlockSimulator::integrate(void)
{
    Vector3 v_op;
    
    v = v_prev + a * h;
    
    if(vortex != nullptr)
    {
        v_op = vortex->get_velocity(pos, (v_prev + v)/2);
        Vector3 temp = (v_prev + v)/2;
        pos = pos_prev + v_op * h;
    }
    else
    {
        pos = pos_prev + ((v_prev + v)/2)*h;
    }
}

void FlockSimulator::integrate_boid(void)
{
    v = v_prev + a * h;
    pos = pos_prev + ((v_prev + v)/2)*h;
}

void FlockSimulator::process_collision(void)
{
    bool collision = check_collision();
    if (collision)
    {
        update_collision_response();
    }
}

bool FlockSimulator::check_collision(void)
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

void FlockSimulator::update_collision_response(void)
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

void FlockSimulator::add_generator(std::string type, const Vector3& center, const Vector3& v, const float& angle, const int& rate, const float& m, const float& surface_radius, const float& air_resistance, const int& life_span, const Vector3& start_color, const Vector3& end_color)
{
    if (type.compare("nozzle") == 0)
    {
        ParticleGenerator* generator = new Nozzle(center, v, angle, rate, m, surface_radius, air_resistance, life_span, start_color, end_color);
        generators.push_back(generator);
    }
}

void FlockSimulator::add_obstacle(std::string type, const Vector3& c_val, const float& r_val, const float& safe_distance_val, const float& t_c_val)
{
    if (type.compare("sphere") == 0)
    {
        obstacles.push_back(new SphereObstacle(c_val, r_val, safe_distance_val, t_c_val));
    }
}

FlockSimulator::~FlockSimulator()
{

}
