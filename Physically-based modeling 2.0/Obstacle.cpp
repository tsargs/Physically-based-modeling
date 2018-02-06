//
//  Obstacle.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Obstacle.h"

Obstacle::Obstacle()
{
    
}

Obstacle::~Obstacle()
{
    
}

Vector3 Obstacle::get_steering_acceleration(const Vector3 &pos, const Vector3 &v, const Vector3 &a)
{
    return Vector3(0,0,0);
}

Vector3 Obstacle::get_position(void)
{
    return Vector3();
}

float Obstacle::get_radius(void)
{
    return 0;
}
