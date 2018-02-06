//
//  SPHParticle.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef SPHPARTICLE
#define SPHPARTICLE

#include <stdio.h>
#include "Matrix4X4.h"

class SPHParticle
{
    public:
        bool is_active;
        bool is_resting;
        bool belongs_to_inactive_particles;
    
        Vector3 pos;
        Vector3 v;
        Vector3 a;
        float m;
        float d;    // constant to account for all the factors that affect air resistance
        double density;
        double pressure;
    
        int voxel_id;
        int p, r ,c;    // voxel indices
                        // p => z plane(depth)
                        // r => row
                        // c => column
    
        float psi;  // banking angle
    
        Vector3 color;
    
        bool is_streak;
        Vector3 streak_start_pos;
    
        bool is_paper_plane;
    
        //Variables for transforming points from local frame to world frame
    
        Matrix4X4 l_w;  // matrix to transform a point from local frame to world frame
    
        Matrix4X4 T;    // translation matrix
        Matrix4X4 R;    // rotation matrix
        Matrix4X4 R_x;  // rotation matrix (about x-axis)
    
        Vector3 u_x, u_y, u_z;
        Vector3 a_v; //acceleration component in the direction of v
        Vector3 a_t; //acceleration component orthogonal to the direction of v
    
        SPHParticle(void);
        ~SPHParticle(void);
    
        bool update_status(void);
        void initialize_matrices(void);
        void compute_banking_and_transformation_matrix(const float& k_banking, const float& banking_smoothing_constant);
        Vector3 transform_point(const Vector3& p);
};

#endif
