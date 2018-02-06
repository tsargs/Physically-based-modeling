//
//  TorsionalSpring.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef TORSIONALSPRING
#define TORSIONALSPRING

#include <stdio.h>

class TorsionalSpring
{
    public:
        int i0, i1, i2, i3; // indices of the vertices that form the faces
    
        float theta0;   // rest angle
    
        TorsionalSpring(void);
        TorsionalSpring(const int& i0_val, const int& i1_val, const int& i2_val, const int& i3_val, const float& rest_angle);
        ~TorsionalSpring(void);
};

#endif
