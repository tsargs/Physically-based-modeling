//
//  SphereObstacle.hpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef SPHEREOBSTACLE
#define SPHEREOBSTACLE

#include <stdio.h>
#include "Obstacle.h"

class SphereObstacle: public Obstacle
{
    public:
        Vector3 c;
        float r;
        float safe_distance;
        float t_c;  // threshold time to be concerned about collision
    
        //variables for steering acceleration calculation
        float R;
        Vector3 vector_pos_c;
        float s_close;          // distance along vector_pos_c that is closest to the sphere center
        Vector3 pos_close;      // point along vector_pos_c that is closest to the sphere center
        float d_c;              // threshold distance until which collision need not be concerned about
        float d;                // distance of pos_close from sphere center
        Vector3 v_unit_vector;
        Vector3 pos_t;          // point along the tangent from pos to the sphere of radius 'R'
        Vector3 v_orthogonal;   // unit vector in the direction orthogonal to v towards pos_t
        float d_t;              // distance of particle from pos_t
        float v_t;              // speed with which the particle is approaching pos_t
        float t_t;              // time to reach pos_t
        float delta_v_s;        // increased average speed needed in the direction orthogonal to current velocity to reach pos_t in time t_t
        float a_s;              // magnitude of required steering acceleration
        float e;                // component of total accleration in the direction of v_orthogonal
    
        SphereObstacle();
        SphereObstacle(const Vector3& c_val, const float& r_val, const float& safe_distance_val, const float& t_c_val);
        ~SphereObstacle();
    
        Vector3 get_steering_acceleration(const Vector3& pos, const Vector3& v, const Vector3& a);
        Vector3 get_position(void);
        float get_radius(void);
};
#endif
