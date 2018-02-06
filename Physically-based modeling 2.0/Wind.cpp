//
//  Wind.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Wind.h"

Wind::Wind(void) : s(Vector3(0,0,0)), v(Vector3(0,0,0)), max_angle(45), max_distance(50)
{
    min_dot_product = cos(max_angle*3.14/180);
}

Wind::Wind(const Vector3& s_new, const Vector3& v_new, const float& new_angle, const float& new_distance, const bool& wind_push_and_pull): s(s_new), v(v_new), max_angle(new_angle), max_distance(new_distance), push_and_pull(wind_push_and_pull)
{
    min_dot_product = cos(max_angle*3.14/180);
}

Wind::~Wind()
{

}

Vector3 Wind::get_wind_velocity(Vector3 obj_position)
{
    Vector3 obj_direction = (obj_position - s).unit_vector();
    Vector3 wind_direction = v.unit_vector();
    
    float dot_product = obj_direction*wind_direction;
    
    int max_dot_product = 1;
    
    float dir_intensity = 1 - (max_dot_product - dot_product) / (max_dot_product - min_dot_product);
    
    float obj_distance = obj_position.distance(s);
    float min_distance = 0.2;
    
    float dist_intensity = (max_distance - obj_distance) / (max_distance-min_distance);
    
    if (!push_and_pull)
    {
        if (dir_intensity < 0)
            dir_intensity = 0;
        
        if (dist_intensity < 0)
            dist_intensity = 0;
    }
    
    return v * (dist_intensity+dir_intensity)/2;
}
