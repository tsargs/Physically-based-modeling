//
//  Matrix.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/1/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef MATRIX
#define MATRIX

#include <stdio.h>
#include <vector>
#include "Vector3.h"

class Matrix
{
    public:
        std::vector<std::vector<double>> elements;
        int w, h;
    
        Matrix(void);
        Matrix(const int& width, const int& height);
    
        ~Matrix();
    
        void add_vector3_as_column(const Vector3& v, const int& column);
        Vector3 multiply(const Vector3& v);
        void print(void);
};

#endif
