//
//  Matrix4X4.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/17/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef MATRIX4X4
#define MATRIX4X4

#include <stdio.h>
#include <vector>
#include "Quaternion.h"

class Matrix4X4
{
    public:
        std::vector<std::vector<double>> elements;
    
        Matrix4X4(void);
    
        ~Matrix4X4();
    
        void set_translation(const Vector3& p);
        void set_rotation_x(const float& theta);
        void add_vector3_as_column(const Vector3& v, const int& column);
        Matrix4X4 operator*(const Matrix4X4& m2);
        Vector3 multiply_with_point3d(const Vector3& v);
        Vector3 get_euler_rotation(void);
        void from_quaternion(const Quaternion& q);
        void print(void);
};

#endif
