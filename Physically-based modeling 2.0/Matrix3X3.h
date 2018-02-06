//
//  Matrix3X3.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef MATRIX3X3
#define MATRIX3X3

#include <stdio.h>
#include <vector>
#include "Vector3.h"

class Matrix3X3
{
    public:
        std::vector<std::vector<double>> elements;
    
        Matrix3X3(void);
    
        ~Matrix3X3();
    
        void set_rotation_x(const float& theta);
        void add_vector3_as_column(const Vector3& v, const int& column);
        Matrix3X3 operator*(const double& s) const;   // multiply by scalar
        Vector3   operator*(const Vector3& v);        // multiply by Vector3
        Matrix3X3 operator=(const Matrix3X3& m2);     // assignment
        Matrix3X3 operator*(const Matrix3X3& m2);
        Matrix3X3 inverse_matrix(void);
        Matrix3X3 transpose(void);
        Vector3 get_euler_rotation(void);
        void print(void);
};

#endif
