//
//  Wind.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef WIND
#define WIND

#include <stdio.h>
#include "Vector3.h"

class Wind
{
    public:
        Vector3 s;  //wind source
        Vector3 v;  //wind velocity
    
        float max_angle;
        float max_distance;
        float min_dot_product;
    
        bool push_and_pull;
    
        Wind(void);
        Wind(const Vector3& s_new, const Vector3& v_new, const float& new_angle, const float& new_distance, const bool& wind_push_and_pull);
        ~Wind();
    
        Vector3 get_wind_velocity(Vector3 obj_position);
};

#endif
