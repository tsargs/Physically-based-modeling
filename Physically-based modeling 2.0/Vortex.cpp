//
//  Vortex.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/4/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Vortex.h"

Vortex::Vortex(void)
{
    
}

Vortex::Vortex(const Vector3& center, const Vector3 axis_dir, const float& l, const float& r, const float& r_f_val, const float& max_r_f_val, const float& tightness): base_center(center), axis(axis_dir), length(l), radius(r), r_f(r_f_val), max_r_f(max_r_f_val), t(tightness)
{
    
}

Vortex::~Vortex(void)
{
    
}

Vector3 Vortex::get_velocity(const Vector3& pos, const Vector3& v)
{
    Vector3 c_pos = pos - base_center;
    
    float projection_length = axis*c_pos;
    
    Vector3 orthogonal_vector = c_pos - axis*projection_length;
    
    float dist = orthogonal_vector.magnitude();
    
    float f = pow(radius/dist, t) * r_f;    //rotational frequency
    
    f = std::min(max_r_f, f);
    
    float angular_velocity = 2*3.14*f;  //angular velocity
    
    //rotate v about 'axis' by an angle angular_velocity
    Vector3 a = v;
    Vector3 b = axis;
    
    Vector3 a_par_b = b*(a*b)/(b*b);
    Vector3 a_per_b = a - a_par_b;
    
    Vector3 w = b.cross(a_per_b);
    
    float x1 = cos(angular_velocity)/a_per_b.magnitude();
    float x2 = sin(angular_velocity)/w.magnitude();
    
    Vector3 a_per_b_theta = ((a_per_b * x1) + (w * x2)) * a_per_b.magnitude();
    
    return a_per_b_theta + a_par_b;
}
