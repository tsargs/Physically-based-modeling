//
//  SphereObstacle.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SphereObstacle.h"

SphereObstacle::SphereObstacle()
{
    
}

SphereObstacle::SphereObstacle(const Vector3& c_val, const float& r_val, const float& safe_distance_val, const float& t_c_val): c(c_val), r(r_val), safe_distance(safe_distance_val), t_c(t_c_val), R(r+safe_distance)
{
    
}

SphereObstacle::~SphereObstacle()
{
    
}

Vector3 SphereObstacle::get_steering_acceleration(const Vector3 &pos, const Vector3 &v, const Vector3 &a)
{
    vector_pos_c = c-pos;
    v_unit_vector = v.unit_vector();
    s_close = vector_pos_c * v_unit_vector;
    
    if (s_close < 0)
        return Vector3(0,0,0);
    
    d_c = v.magnitude()*t_c;
    
    if (s_close > d_c)
        return Vector3(0,0,0);
    
    pos_close = pos + (v_unit_vector * s_close);
    d = (pos_close - c).magnitude();
    
    if(d > R)
        return Vector3(0,0,0);
    
    v_orthogonal = (pos_close - c).unit_vector();
    pos_t = c + (v_orthogonal * R);
    Vector3 vector_pos_pos_t = pos_t - pos;
    d_t = vector_pos_pos_t.magnitude();
    v_t = v * (vector_pos_pos_t / d_t);
    t_t = d_t/v_t;
    
    delta_v_s = v_unit_vector.cross(vector_pos_pos_t).magnitude()/t_t;
    a_s = 2 * delta_v_s / t_t;
    
    e = v_orthogonal * a;
    
    return v_orthogonal * std::max((float)(a_s-e), (float)0.0);
}

Vector3 SphereObstacle::get_position(void)
{
    return c;
}

float SphereObstacle::get_radius(void)
{
    return r;
}
