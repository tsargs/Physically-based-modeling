//
//  Vector2.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Vector2.h"

Vector2::Vector2(void) : x(0), y(0)
{
    
}

Vector2::Vector2(const double& xVal, const double& yVal) : x(xVal), y(yVal)
{
    
}

Vector2::Vector2(const Vector2& v) : x(v.x), y(v.y)
{
    
}

Vector2::~Vector2(void)
{
    
}
