//
//  SPHParticle.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SPHParticle.h"

SPHParticle::SPHParticle(void)
{
    is_active = false;
    belongs_to_inactive_particles = false;
}

SPHParticle::~SPHParticle(void)
{
    
}

void SPHParticle::compute_banking_and_transformation_matrix(const float& k_banking, const float& banking_smoothing_constant)
{
    ///*
    u_x = v.unit_vector();
    u_z = u_x.cross(Vector3(0,1,0));
    u_y = u_z.cross(u_x);
    //*/
    
    a_v = u_x * (a * u_x);
    a_t = a - a_v;
    
    float new_psi = atan(k_banking * (a_t * u_z));
    
    psi = (1-banking_smoothing_constant) * psi + banking_smoothing_constant * new_psi;
    
    T.set_translation(pos);
    R.add_vector3_as_column(u_x, 0);
    R.add_vector3_as_column(u_y, 1);
    R.add_vector3_as_column(u_z, 2);
    R.elements[3][3] = 1;
    R_x.set_rotation_x(psi);
    
    l_w = T*(R)*(R_x);
}

Vector3 SPHParticle::transform_point(const Vector3& p)
{
    return l_w.multiply_with_point3d(p);
}
