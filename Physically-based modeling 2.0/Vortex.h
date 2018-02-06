//
//  Vortex.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/4/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef VORTEX
#define VORTEX

#include <stdio.h>
#include <algorithm>
#include "Vector3.h"

class Vortex
{
    public:
        Vector3 base_center;
        Vector3 axis;
        float length;
        float radius;
        float r_f;
        float max_r_f;
        float t;
    
        Vortex(void);
        Vortex(const Vector3& center, const Vector3 axis_dir, const float& l, const float& r, const float& r_f_val, const float& max_r_f_val, const float& tightness);
    
        ~Vortex(void);
    
        Vector3 get_velocity(const Vector3& pos, const Vector3& v);
};

#endif
