//
//  SPHSimulator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/01/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SPHSimulator.h"


SPHSimulator::SPHSimulator(void)
{
    
}

SPHSimulator::SPHSimulator(const int& count):max_particles(count)
{
    for (int i = 0; i < max_particles; i++)
    {
        //inactive_particles.push(i);
        particles.push_back(SPHParticle());
    }
    
    for(int i = 0; i < 10; i++)
    {
        trail_positions[i] = pos;
    }
    
    reference_density = 1000;   // for water
    kinematic_viscosity = 1;
    support_radius = 20;
    support_radius_sq = support_radius * support_radius;
    neighbor_offset = 6;
    stiffness_parameter = 1000;
}


SPHSimulator::~SPHSimulator()
{
    
}

void SPHSimulator::initialize_uniform_grid(const Vector3& min, const Vector3& dimensions, const Vector3& resolution)
{
    min_bound = min;
    
    W = dimensions.x;
    H = dimensions.y;
    D = dimensions.z;
    
    L = resolution.z;
    M = resolution.y;
    N = resolution.x;
    
    voxel_particles.resize(N * M * L);
    
    voxel.x = W/N;
    voxel.y = H/M;
    voxel.z = D/L;
}

int SPHSimulator::get_voxel_index(const Vector3 position)
{
    p = floor((position.z - min_bound.z)/voxel.z);     // p => z plane(depth)
    r = floor((position.y - min_bound.y)/voxel.y);     // r => row
    c = floor((position.x - min_bound.x)/voxel.x);     // c => column
    
    return (p*M + r)*N + c;
}

bool SPHSimulator::check_index_bounds(const int& u, const int& v, const int& w)
{
    if(u < 0 || u >= L)
        return false;
    
    if(v < 0 || v >= M)
        return false;
    
    if(w < 0 || w >= N)
        return false;
    
    return true;
}

void SPHSimulator::clear_voxels(void)
{
    for(int i = 0; i < voxel_particles.size(); i++)
    {
        voxel_particles[i].clear();
    }
}

void SPHSimulator::update_generator_transform(const Vector3& position, const Vector3& normal)
{
    generators[0]->update_generator_transform(position, normal);
}

void SPHSimulator::update_simulation(const double& displayStep, const bool& generate_particles)
{
    double t = 0;
        
    while (t < displayStep)
    {
        /*
        if(generate_particles)
        {
            for (int i = 0 ; i < generators.size(); i ++)
            {
                generators[i]->generate_particles(h, particles, inactive_particles);
            }
        }
         */
        
        if(!inactive_particles.empty())
            generators[0]->generate_particles_in_grid(particles, inactive_particles);
        
        // add particles to the voxels
        clear_voxels();
        for (int i = 0 ; i < particles.size(); i++)
        {
            if(particles[i].is_active)
            {
                index = get_voxel_index(particles[i].pos);
                
                particles[i].voxel_id = index;
                particles[i].p = p;
                particles[i].r = r;
                particles[i].c = c;
                
                if(index >=0 && index < voxel_particles.size())
                    voxel_particles[index].push_back(i);
            }
        }
        
        // calculate prerequisites (density and pressure)
        for (int i = 0 ; i < particles.size(); i++)
        {
            if(particles[i].is_active)
            {
                p = particles[i].p;
                r = particles[i].r;
                c = particles[i].c;
                
                particles[i].density = 0;
                
                for(int u = p-neighbor_offset; u <= p+neighbor_offset; u++)
                {
                    for(int v = r-neighbor_offset; v <= r+neighbor_offset; v++)
                    {
                        for(int w = c-neighbor_offset; w <= c+neighbor_offset; w++)
                        {
                            if(check_index_bounds(u,v,w))
                            {
                                int n = (u*M + v)*N + w;    // neighboring voxel
                                
                                for(int t = 0; t < voxel_particles[n].size(); t++)
                                {
                                    int j = voxel_particles[n][t];
                                    Vector3 r = (particles[i].pos - particles[j].pos);
                                    double r_length = r.magnitude();
                                    
                                    float monaghan_kernel = 0;
                                    
                                    if(r_length <= support_radius)
                                    {
                                        monaghan_kernel = (1/(3.14159*pow(support_radius, 3))) * (1 - (3/2)*pow(r_length/support_radius, 2) + (3/4)*pow(r_length/support_radius, 3));
                                    }
                                    else if (r_length > support_radius && r_length <= 2*support_radius)
                                    {
                                        monaghan_kernel = (1/(3.14159*pow(support_radius, 3))) * ((1/4)*pow(2 - r_length/support_radius, 3));
                                    }
                                    
                                    particles[i].density += particles[j].m * monaghan_kernel;
                                }
                            }
                        }
                    }
                }
                particles[i].pressure = stiffness_parameter*(particles[i].density - reference_density);
            }
        }
        
        for (int i = 0 ; i < particles.size(); i++)
        {
            if(particles[i].is_active)
            {
                if(particles[i].is_resting)
                    continue;
                
                pos = particles[i].pos;
                v   = particles[i].v;
                a   = particles[i].a;
                d   = particles[i].d;
                m   = particles[i].m;
                
                pos_prev = pos;
                v_prev = v;
                
                pressure_gradient = Vector3(0,0,0);
                laplacian = Vector3(0,0,0);
                
                for(int u = p-neighbor_offset; u <= p+neighbor_offset; u++)
                {
                    for(int v = r-neighbor_offset; v <= r+neighbor_offset; v++)
                    {
                        for(int w = c-neighbor_offset; w <= c+neighbor_offset; w++)
                        {
                            if(check_index_bounds(u,v,w))
                            {
                                int n = (u*M + v)*N + w;    // neighboring voxel
                                for(int t = 0; t < voxel_particles[n].size(); t++)
                                {
                                    int j = voxel_particles[n][t];
                                    
                                    Vector3 r = (particles[i].pos - particles[j].pos);
                                    double r_length = r.magnitude();
                                    
                                    //compute pressure gradient
                                    Vector3 grad_spiky_kernel = Vector3(0,0,0);
                                    
                                    if (r_length <= support_radius)
                                        grad_spiky_kernel = r.unit_vector() * (-45/(3.14159*pow(support_radius, 6))) * pow(support_radius - r_length, 2);
                                    
                                    double pressure_sum = (particles[i].pressure/pow(particles[i].density, 2))
                                                         +(particles[j].pressure/pow(particles[j].density, 2));
                                    pressure_gradient = pressure_gradient + (grad_spiky_kernel * particles[j].m * pressure_sum);
                                    
                                    //compute laplacian of the velocity field
                                    double laplacian_poly_kernel = 0;
                                    
                                    if(r_length <= support_radius)
                                        laplacian_poly_kernel = (45/(3.14159*pow(support_radius, 6)))*(support_radius - r_length);
                                    
                                    laplacian = laplacian + ((particles[j].v - particles[i].v)/particles[i].density) * particles[j].m * laplacian_poly_kernel;
                                }
                            }
                        }
                    }
                }
                
                //compute acceleration for particle i
                a = -pressure_gradient + laplacian * kinematic_viscosity + g;
                
                //update_acceleration();
                integrate();
                process_collision();
                
                particles[i].pos = pos;
                particles[i].v = v;
                particles[i].a = a;
                particles[i].is_resting = is_resting;
                
                is_resting = false;
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

void SPHSimulator::update_acceleration(void)
{
    Vector3 v_wind;
    
    for (int i = 0 ; i < winds.size(); i ++)
        v_wind = v_wind + winds[i].get_wind_velocity(pos);
    
    a = g + (v_wind - v) * (d/m);
}

void SPHSimulator::integrate(void)
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
        //pos = pos_prev + ((v_prev + v)/2)*h;
        pos = pos_prev + (v_prev * h);
    }
}

void SPHSimulator::process_collision(void)
{
    f = 1;
    bool collision = check_collision();
    if (collision)
    {
        update_collision_response();
    }
}

bool different_sign(double a, double b)
{
    double tol = 0.0004;
    
    if (a * b < 0)
    {
        if (std::abs(a) < tol || std::abs(b) < tol)
            return false;
        
        return true;
    }
    
    return false;
}

bool SPHSimulator::check_collision(void)
{
    double t_hit_tol = 0.0002;//0.00008;
    bool collision = false;
    
    for (int i = 0; i < polygon_count; i++)
    {
        Vector3 p = polygons[i].vertices[0];
        Vector3 n = polygons[i].normal;
        
        float radius = 3;
        
        double dist = (pos_prev-p)*n;
        
        if (dist >= 0)
            dist -= radius;
        else
            dist += radius;
        
        double new_dist = (pos-p)*n;
        
        if (new_dist >= 0)
            new_dist -= radius;
        else
            new_dist += radius;
        
        if (different_sign(new_dist, dist))
        {
            collision = true;
            double new_f = dist / (dist - new_dist);
            if (new_f <= f)
            {
                f = new_f;
                colliding_polygons.push_back(&polygons[i]);
            }
        }
    }
    
    return collision;
}

void SPHSimulator::update_collision_response(void)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        Vector3 p = colliding_polygons[i]->vertices[0];
        Vector3 n = colliding_polygons[i]->normal;
        double cr = colliding_polygons[i]->cr;
        double cf = colliding_polygons[i]->cf;
        
        float radius = 3;
        double dist = (pos-p)*n;
        
        if (dist >= 0)
            dist -= radius;
        else
            dist += radius;
        
        pos = pos - n * (1 + cr) * dist;
        Vector3 v_normal = n*(v*n);
        Vector3 v_tangential = v - v_normal;
        
        v = v_normal*(-cr) + v_tangential*(1-cf);
    }
    test_resting_conditions();
    colliding_polygons.clear();
}

void SPHSimulator::test_resting_conditions(void)
{
    double v_tol = 6.5;
    double d_tol = 0.1;
    
    if(v.magnitude() < v_tol)
    {
        for (int i = 0; i< colliding_polygons.size(); i++)
        {
            if (colliding_polygons[i] != nullptr  && colliding_polygons[i]->dist < d_tol)
            {
                if(a * colliding_polygons[i]->normal.unit_vector() < 0)
                {
                    is_resting = true;
                    return;
                }
            }
        }
    }
}

void SPHSimulator::add_generator(std::string type, const Vector3& center, const Vector3& v, const float& angle, const int& rate, const float& m, const float& surface_radius, const float& air_resistance, const Vector3& color)
{
    if (type.compare("nozzle") == 0)
    {
        SPHParticleGenerator* generator = new SPHNozzle(center, v, angle, rate, m, surface_radius, air_resistance, color);
        generators.push_back(generator);
    }
}
