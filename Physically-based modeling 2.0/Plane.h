//
//  Plane.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/15/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef PLANE
#define PLANE

#include <stdio.h>
#include "Vector3.h"

class Plane
{
    public:
        Vector3 point;
        Vector3 normal;
    
        double dist;  //distance for collision check
    
        //  coefficient of restitution (fraction of speed with which the object will move away from plane)
        //  0 => inelastic collision; most of the momentum in the normal direction is lost during collision
        //  1 => equal energy added to the colliding object in the normal direction
        double cr;
    
        //***  coefficient of friction (fraction of tangential speed lost during collision)
        //  0 => no friction; very slippery
        //  1 => tangential velocity is completely wiped out; very rough surface
        //***
        double cf;
    
        Plane(void);
    
        void set_point(const Vector3& new_point);
        void set_normal(const Vector3& new_normal);
};

#endif
