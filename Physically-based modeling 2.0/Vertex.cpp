//
//  Vertex.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Vertex.h"

Vertex::Vertex(void)
{
    
}

Vertex::Vertex(const Vector3& position, const float& mass):pos0(position), m(mass)
{
    
}

Vertex::Vertex(const Vector3& position, const float& mass, const Vector3& velocity):pos0(position), m(mass), v(velocity)
{
    
}

Vertex::~Vertex(void)
{
    
}
