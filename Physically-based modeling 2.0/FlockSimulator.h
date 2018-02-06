//
//  FlockSimulator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/01/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef FLOCKSIMULATOR
#define FLOCKSIMULATOR

#include <iostream>
#include <vector>
#include <stack>
#include <cmath>
#include <string>
#include <algorithm>
#include "Polygon.h"
#include "Matrix2X2.h"
#include "Wind.h"
#include "Vortex.h"
#include "Nozzle.h"
#include "SphereObstacle.h"

class FlockSimulator
{
    public:
        std::vector<Particle> particles;
        std::stack<int> inactive_particles;
        std::vector<ParticleGenerator*> generators;
        std::vector<Obstacle*> obstacles;
    
        int count;
        int max_particles;
    
        double h = (double)1/90;
        Vector3 g = Vector3(0, -10, 0);
    
        float ka;   //boid-boid collision avoidance factor
        float kv;   //boid-boid velocity matching factor
        float kc;   //boid-boid centering factor
    
        //distance thresholds
        float r1;
        float r2;
    
        //angle thresholds (stored in radians)
        float theta1;   //frontal binocular
        float theta2;   //peripheral monocular
    
        //variables for flock simulation
        Vector3 a_a;    // acceleration for collision avoidance
        Vector3 a_v;    // acceleration for velocity matching
        Vector3 a_c;    // acceleration for flock centering
    
        float boid_d;       //boid-boid distance
        float boid_theta;   //angle between a boid's velocity vector and its direction vector with another boid
    
        float kd;
        float ktheta;
    
        float a_max = 500;  // maximum acceleration
        float a_r;          // residual acceleration
    
        //variables for banking
        float psi;
        float k_banking;    // constant to account for vertical force (usually gravity; value of +0.1);
        float banking_smoothing_constant;
    
        //-- other dummy variables
        Vector3 pos_i, pos_j;
        Vector3 v_i, v_j;
        float d_i_j;
        Vector3 vector_i_j;
        Vector3 a_a_i_j;
        Vector3 a_v_i_j;
        Vector3 a_c_i_j;
    
        //variables for boid simulation
        Vector3 pos, v, a;
        float d, m;
    
        Vector3 pos_prev, v_prev;
    
        //variables for lead-boid simulation
        bool lead_boid_controlled_by_physics;
        bool lead_boid_controlled_by_sine_function;
        Vector3 lead_boid_start_position;
        float lead_boid_speed;
    
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
    
        Vector3 ZERO_VECTOR = Vector3(0,0,0);
        const float TO_RADIANS_MULTIPLIER = 3.14159265359 / 180;
        const float TO_DEGREES_MULTIPLIER = 180 / 3.14159265359;

    
        //Constructors
        FlockSimulator(void);
        FlockSimulator(const int& count);
    
        //Destructor
        ~FlockSimulator(void);
    
        //Functions
        void update_simulation(const double& displayStep, const bool& generate_particles);
        void update_lead_boid_simulation(void);
        void update_acceleration(void);
        void integrate(void);
        void integrate_boid(void);
        void process_collision(void);
    
        void add_generator(std::string type, const Vector3& pos, const Vector3& v, const float& angle, const int& rate, const float& m, const float& surface_radius, const float& air_resistance, const int& life_span, const Vector3& start_color, const Vector3& end_color);
    
        void add_obstacle(std::string type, const Vector3& c_val, const float& r_val, const float& safe_distance_val, const float& t_c_val);
    
        void update_generator_transform(const Vector3& position, const Vector3& normal);
    
        bool check_collision(void);
        void update_collision_response(void);
};

#endif
