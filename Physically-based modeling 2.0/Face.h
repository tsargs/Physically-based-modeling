//
//  Face.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef FACE
#define FACE

#include <stdio.h>

class Face
{
    public:
    
        int s1, s2, s3; // indices of the three struts that form the triangular face
    
        Face(void);
        Face(const int& indexOne, const int& indexTwo, const int& indexThree);
        ~Face(void);
};

#endif
