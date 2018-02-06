//
//  Particle.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef PARTICLE
#define PARTICLE

#include <stdio.h>
#include "Matrix4X4.h"


class Particle
{
    public:
        bool is_active;
        bool belongs_to_inactive_particles;
    
        Vector3 pos;
        Vector3 v;
        Vector3 a;
        float m;
        float d;    // constant to account for all the factors that affect air resistance
    
        float psi;  // banking angle
    
        Vector3 color;
    
        Vector3 start_color;
        Vector3 end_color;
    
        float life_span;
        float time_left;
    
        bool is_streak;
        Vector3 streak_start_pos;
    
        bool is_paper_plane;
    
        //Variables for transforming points from local frame to world frame
    
        Matrix4X4 l_w;  // matrix to transform a point from local frame to world frame
    
        Matrix4X4 t;    // translation matrix
        Matrix4X4 r;    // rotation matrix
        Matrix4X4 r_x;  // rotation matrix (about x-axis)
    
        Vector3 u_x, u_y, u_z;
        Vector3 a_v; //acceleration component in the direction of v
        Vector3 a_t; //acceleration component orthogonal to the direction of v
    
        Particle(void);
        Particle(const Vector3& position, const Vector3& velocity, const float& life_span, const float& particle_mass, const float& air_resistance, const Vector3& start_col, const Vector3& end_col, const bool& status);
        ~Particle(void);
    
        bool update_status(void);
        void update_color(void);
        void initialize_matrices(void);
        void compute_banking_and_transformation_matrix(const float& k_banking, const float& banking_smoothing_constant);
        Vector3 transform_point(const Vector3& p);
};

#endif
