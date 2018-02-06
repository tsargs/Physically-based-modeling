//
//  Vertex.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef VERTEX
#define VERTEX

#include <stdio.h>
#include "Vector3.h"

class Vertex
{
    public:
        Vector3 pos0;    // local position
        float m;         // mass
        Vector3 v;       // velocity
        Vector3 f;       // force
    
        Vector3 pos;      // world position
    
        Vertex(void);
        Vertex(const Vector3& position, const float& mass);
        Vertex(const Vector3& position, const float& mass, const Vector3& velocity);
        ~Vertex(void);
};

#endif
