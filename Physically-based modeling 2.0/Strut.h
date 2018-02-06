//
//  Strut.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef STRUT
#define STRUT

#include <stdio.h>
#include "Vector3.h"

class Strut
{
    public:
        int v1, v2; // indices of the two vertices that form the strut
    
        float k;    // spring constant
        float d;    // damping constant
        float l;    // resting length
    
        Strut(void);
        Strut(const int& indexOne, const int& indexTwo);
        Strut(const int& indexOne, const int& indexTwo, const float& k_val, const float& d_val, const float& base_length, const float& l_val);
    
        ~Strut(void);
};

#endif
