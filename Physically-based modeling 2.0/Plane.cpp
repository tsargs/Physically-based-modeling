//
//  Plane.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/15/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Plane.h"

Plane::Plane(void) : point(Vector3(0,0,0)), normal(Vector3(0,0,0)), cr(1), cf(0.3), dist(0)
{

}

void Plane::set_point(const Vector3& new_point)
{
    point = new_point;
}

void Plane::set_normal(const Vector3& new_normal)
{
    normal = new_normal;
}