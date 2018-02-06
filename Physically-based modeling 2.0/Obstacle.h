//
//  Obstacle.hpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef OBSTACLE
#define OBSTACLE

#include <stdio.h>
#include "Vector3.h"

class Obstacle
{
    public:
        Obstacle();
        ~Obstacle();
    
        virtual Vector3 get_steering_acceleration(const Vector3& pos, const Vector3& v, const Vector3& a);
        virtual Vector3 get_position(void);
        virtual float get_radius(void);
};

#endif
