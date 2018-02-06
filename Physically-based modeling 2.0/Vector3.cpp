//
//  Vector3.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/15/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Vector3.h"

Vector3::Vector3(void) : x(0), y(0), z(0)
{

}

Vector3::Vector3(const double& xVal, const double& yVal, const double& zVal) : x(xVal), y(yVal), z(zVal)
{
    
}

Vector3::Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z)
{
    
}

Vector3::~Vector3(void)
{

}