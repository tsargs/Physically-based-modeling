//
//  ParticleSimulator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/01/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef PARTICLESIMULATOR
#define PARTICLESIMULATOR

#include <iostream>
#include <vector>
#include <stack>
#include <cmath>
#include <string>
#include "Polygon.h"
#include "Matrix2X2.h"
#include "Wind.h"
#include "Vortex.h"
#include "Nozzle.h"

class ParticleSimulator
{
    public:
        std::vector<Particle> particles;
        std::stack<int> inactive_particles;
        std::vector<ParticleGenerator*> generators;
    
        int count;
        int max_particles;
    
        double h = (double)1/90;
        Vector3 g = Vector3(0, -10, 0);
    
        Vector3 pos, v, a;
        float d, m;
    
        Vector3 pos_prev, v_prev;
    
        Vector3 ambient_color;
        Vector3 diffuse_color;
    
        Polygon *polygons;
        int polygon_count;
        std::vector <Wind> winds;
    
        Vortex* vortex = nullptr;

        //Collision handling
        double t_hit;
        std::vector<Polygon*> colliding_polygons;
    
        //Trail positions
        Vector3 trail_positions[10];
        int current_trail_position_index = 0;
        int time_step_counter = 0;
    

        ParticleSimulator(void);
        ParticleSimulator(const int& count);
    
        ~ParticleSimulator(void);
    
        void update_simulation(const double& displayStep, const bool& generate_particles);
        void update_acceleration(void);
        void integrate(void);
        void process_collision(void);
    
        void add_generator(std::string type, const Vector3& pos, const Vector3& v, const float& angle, const int& rate, const float& m, const float& surface_radius, const float& air_resistance, const int& life_span, const Vector3& start_color, const Vector3& end_color);
    
        void update_generator_transform(const Vector3& position, const Vector3& normal);
    
        bool check_collision(void);
        void update_collision_response(void);
};

#endif
