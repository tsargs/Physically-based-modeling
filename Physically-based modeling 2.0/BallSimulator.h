//
//  Simulator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/16/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef BALLSIMULATOR
#define BALLSIMULATOR

#include <iostream>
#include <vector>
//#include <algorithm>
#include <cmath>
#include "Polygon.h"
#include "Matrix2X2.h"
#include "Wind.h"
#include "Vortex.h"


class BallSimulator
{
    public:
        Vector3 pos;
        Vector3 v;
        float m;
        float radius;
        float d;    // constant to account for all the factors that affect air resistance
    
        double h = (double)1/90;
        Vector3 g = Vector3(0, -10, 0);
        Vector3 a;
    
        Vector3 ambient_color;
        Vector3 diffuse_color;
    
        Polygon *polygons;
        int polygon_count;
        std::vector <Wind> winds;
    
        bool ball_resting = false;
    
        Vortex* vortex = nullptr;
    
        //Collision handling
        double t_hit;
        std::vector<Polygon*> colliding_polygons;
        std::vector<BallSimulator> other_balls;
        bool ball_collision;
    
        //Trail positions
        Vector3 trail_positions[10];
        int current_trail_position_index = 0;
        int time_step_counter = 0;
    
        Vector3 v_prev;
        Vector3 pos_prev;
    
        BallSimulator(void);
        BallSimulator(const Vector3& new_pos, const Vector3& new_v, const float& new_mass, const float& new_radius, const float& new_d);
    
        ~BallSimulator(void);
    
        void update_simulation(double displayStep);
        void integrate(const double& timestep);
        bool check_collision(void);
        void update_collision_response(void);
        void test_resting_conditions(void);
        void update_trail_positions(void);
    
        void check_ball_collision(void);
};

#endif
