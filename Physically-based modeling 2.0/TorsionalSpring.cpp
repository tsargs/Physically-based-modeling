//
//  TorsionalSpring.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "TorsionalSpring.h"

TorsionalSpring::TorsionalSpring(void)
{
    
}

TorsionalSpring::TorsionalSpring(const int& i0_val, const int& i1_val, const int& i2_val, const int& i3_val, const float& rest_angle):i0(i0_val), i1(i1_val), i2(i2_val), i3(i3_val), theta0(rest_angle)
{
    
}

TorsionalSpring::~TorsionalSpring(void)
{
    
}
