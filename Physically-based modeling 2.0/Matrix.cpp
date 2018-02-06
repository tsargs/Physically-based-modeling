//
//  Matrix.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Matrix.h"

Matrix::Matrix(void)
{
    
}

Matrix::Matrix(const int& width, const int& height): w(width), h(height)
{
    elements.resize(h);
    for (int i = 0; i < h; i++)
        elements[i].resize(w);
}

Matrix::~Matrix()
{
    
}

void Matrix::add_vector3_as_column(const Vector3& v, const int& column)
{
    elements[0][column] = v.x;
    elements[1][column] = v.y;
    elements[2][column] = v.z;
}

Vector3 Matrix::multiply(const Vector3& v)
{
    Vector3 result;
    if (w == 3)
    {
        result.x = elements[0][0]*v.x + elements[0][1]*v.y + elements[0][2]*v.z;
        result.y = elements[1][0]*v.x + elements[1][1]*v.y + elements[1][2]*v.z;
        result.z = elements[2][0]*v.x + elements[2][1]*v.y + elements[2][2]*v.z;
    }
    return result;
}

void Matrix::print(void)
{
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            std::cout << elements[i][j] << ", ";
        }
        std::cout << "\n";
    }
}
