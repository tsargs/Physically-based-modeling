//
//  Strut.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Strut.h"

Strut::Strut(void)
{
    
}

Strut::Strut(const int& indexOne, const int& indexTwo, const float& k_val, const float& d_val, const float& base_length, const float& l_val):v1(indexOne), v2(indexTwo), l(l_val)
{
    k = (base_length/l)*k_val;
    d = (base_length/l)*d_val;
}

Strut::Strut(const int& indexOne, const int& indexTwo):v1(indexOne), v2(indexTwo)
{
    
}

Strut::~Strut(void)
{
    
}
