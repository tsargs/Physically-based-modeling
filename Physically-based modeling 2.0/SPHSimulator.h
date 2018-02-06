//
//  SPHSimulator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 12/10/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef SPHSIMULATOR
#define SPHSIMULATOR

#include <iostream>
#include <vector>
#include <stack>
#include <cmath>
#include <string>
#include "Polygon.h"
#include "Matrix2X2.h"
#include "Wind.h"
#include "Vortex.h"
#include "SPHNozzle.h"

class SPHSimulator
{
    public:
        std::vector<SPHParticle> particles;
        std::stack<int> inactive_particles;
        std::vector<SPHParticleGenerator*> generators;
    
        int count;
        int max_particles;
    
        Vector3 pressure_gradient;
        Vector3 laplacian;
        Vector3 external_forces;
    
        double reference_density;
        double kinematic_viscosity;
        double support_radius;
        double support_radius_sq;
        double stiffness_parameter;
    
        //Uniform spatial grid
        std::vector<std::vector<int>> voxel_particles;
        double W, H, D;    // width, height, depth
        int L, M, N;    // L => depth layers
                        // M => number of rows
                        // N => number of columns
        Vector3 voxel;  // voxel dimension
        Vector3 min_bound;
        int p, r ,c;    // voxel indices
                        // p => z plane(depth)
                        // r => row
                        // c => column
        int index;      // 1 dimensional index
        int neighbor_offset;
    
        double h = (double)1/90;
        //double h = (double)1/300;
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
        double f;
        double ft;
        std::vector<Polygon*> colliding_polygons;
        bool is_resting;
    
        //Trail positions
        Vector3 trail_positions[10];
        int current_trail_position_index = 0;
        int time_step_counter = 0;

        SPHSimulator(void);
        SPHSimulator(const int& count);
    
        ~SPHSimulator(void);
    
        void update_simulation(const double& displayStep, const bool& generate_particles);
        void update_acceleration(void);
        void integrate(void);
        void process_collision(void);
    
        void initialize_uniform_grid(const Vector3& min, const Vector3& dimensions, const Vector3& resolution);
        int get_voxel_index(const Vector3 position);
        bool check_index_bounds(const int& u, const int& v, const int& w);
        void clear_voxels(void);
    
        void add_generator(std::string type, const Vector3& pos, const Vector3& v, const float& angle, const int& rate, const float& m, const float& surface_radius, const float& air_resistance, const Vector3& end_color);
    
        void update_generator_transform(const Vector3& position, const Vector3& normal);
    
        bool check_collision(void);
        void update_collision_response(void);
        void test_resting_conditions(void);
};

#endif
