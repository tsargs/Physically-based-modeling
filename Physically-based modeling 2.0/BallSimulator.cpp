//
//  BallSimulator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/16/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "BallSimulator.h"


BallSimulator::BallSimulator()
{
    for(int i = 0; i < 10; i++)
    {
        trail_positions[i] = pos;
    }
}

BallSimulator::BallSimulator(const Vector3& new_pos, const Vector3& new_v, const float& new_mass, const float& new_radius, const float& new_d): pos(new_pos), v(new_v), m(new_mass), radius(new_radius), d(new_d)
{
    for(int i = 0; i < 10; i++)
    {
        trail_positions[i] = pos;
    }
}

void BallSimulator::update_simulation(double displayStep)
{
    if (!ball_resting)
    {
        double t = 0;
        
        while (t < displayStep)
        {
            double h_total = 0;
            double h_current = h;
            
            Vector3 v_wind;
            
            for (int i = 0 ; i < winds.size(); i ++)
                v_wind = v_wind + winds[i].get_wind_velocity(pos);
            
            a = g + (v_wind - v) * (d/m);
            
            if (ball_resting)
                return;
            
            while (h_total < h)
            {
                v_prev = v;
                pos_prev = pos;
                
                t_hit = h_current;
                
                //integrate(h_current);
                
                //Collision detection
                ball_collision = false;
                bool collision = check_collision();
                
                if (collision)
                {
                    //Collision determination
                    //t_hit already has the least value of all collisions
                   
                    //Collision response
                    integrate(t_hit);
                    update_collision_response();
                    
                    h_total += t_hit;
                    t += t_hit;
                    
                    h_current = h - (t_hit);
                }
                else
                {
                    integrate(h_current);
                    h_total += h;
                    t += h;
                }
            }
        }
        update_trail_positions();
    }
}

void BallSimulator::update_trail_positions(void)
{
    time_step_counter++;
    
    float dist = pos.distance(trail_positions[0]);
    
    if (dist > 7.5 || (time_step_counter >= 8 && dist < 7.5))
    {
        time_step_counter = 0;
        for(int i = 9; i > 0; i--)
        {
            trail_positions[i] = trail_positions[i-1];
        }
        trail_positions[0] = pos;
    }
}

bool BallSimulator::check_collision(void)
{
    double t_hit_tol = 0.00008;// 0.0002;
    bool collision = false;
    
    for (int i = 0; i < polygon_count; i++)
    {
        Vector3 p = polygons[i].vertices[0];
        Vector3 n = polygons[i].normal;
        
        //Check collision with the plane that has the polygon
        Vector3 pos_adjusted = pos - n*radius;
        double t_hit_new = ((p - pos_adjusted)*n)/(v*n);
        
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
                else if(t_hit_new == t_hit)
                {
                    colliding_polygons.push_back(&polygons[i]);
                }
                
                collision = true;
            }
        }
        
        /*
        if (collision == false)
        {
            //check collision with other balls
            check_ball_collision();
        }
         */
    }
    
    return collision;
}

void BallSimulator::check_ball_collision(void)
{
    for (int i = 0; i < other_balls.size(); i++)
    {
        double dist = pos.distance(other_balls[i].pos);
        other_balls[i].pos.print();
        
        if(dist <= radius + other_balls[i].radius)
        {
            //collision with another ball
            v = (v + other_balls[i].v)/2;
            ball_collision = true;
        }
    }
}

void BallSimulator::test_resting_conditions(void)
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
                    ball_resting = true;
                    return;
                }
            }
        }
    }
}

void BallSimulator::integrate(const double& timestep)
{
    Vector3 v_op;
    
    v = v_prev + a * timestep;
    
    if(vortex != nullptr)
    {
        v_op = vortex->get_velocity(pos, (v_prev + v)/2);
        Vector3 temp = (v_prev + v)/2;
        pos = pos_prev + v_op * timestep;
    }
    else
    {
        pos = pos_prev + ((v_prev + v)/2)*timestep;
    }
}

void BallSimulator::update_collision_response(void)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        v_prev = v;
        Vector3 n = colliding_polygons[i]->normal;
        
        Vector3 v_normal_prev = n*(v_prev*n);               // Velocity in the direction of plane's normal
        Vector3 v_tangential_prev = v_prev - v_normal_prev; // Velocity in the direction of plane's surface
        
        Vector3 v_normal = v_normal_prev * (-colliding_polygons[i]->cr);
        Vector3 v_tangential = v_tangential_prev * (1- colliding_polygons[i]->cf);
        
        //Vector3 v_tangential = v_tangential_prev - (v_tangential_prev.unitVector() * std::max((1 - colliding_plane->cf) * v_normal_prev.magnitude(), v_tangential_prev.magnitude()));
        //Vector3 v_tangential = v_tangential_prev - (v_tangential_prev.unitVector() * std::min(colliding_plane->cf * m * v_normal_prev.magnitude(), v_tangential_prev.magnitude()));
        
        v = v_normal + v_tangential;
        
        Vector3 pos_adjusted = pos - n*radius;
        colliding_polygons[i]->dist = (pos_adjusted - colliding_polygons[i]->vertices[0]) * n;
        
    }
    test_resting_conditions();
    colliding_polygons.clear();
}

BallSimulator::~BallSimulator()
{

}
