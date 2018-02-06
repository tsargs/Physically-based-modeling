//
//  Matrix4X4.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/17/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Matrix4X4.h"

Matrix4X4::Matrix4X4(void)
{
    elements.resize(4);
    for (int i = 0; i < 4; i++)
        elements[i].resize(4);
}

Matrix4X4::~Matrix4X4()
{
    
}

void Matrix4X4::set_translation(const Vector3& p)
{
    //elements[0][0] = 1; elements[0][1] = 0; elements[0][2] = 0; elements[0][3] = p.x;
    elements = {{1,0,0,p.x},
                {0,1,0,p.y},
                {0,0,1,p.z},
                {0,0,0,1}};
}

void Matrix4X4::set_rotation_x(const float& theta)
{
    elements = {{1,0,0,0},
                {0,cos(theta),-sin(theta),0},
                {0,sin(theta),cos(theta),0},
                {0,0,0,1}};
}

void Matrix4X4::add_vector3_as_column(const Vector3& v, const int& column)
{
    elements[0][column] = v.x;
    elements[1][column] = v.y;
    elements[2][column] = v.z;
    elements[3][column] = 0;
}

Matrix4X4 Matrix4X4::operator*(const Matrix4X4& m2)
{
    Matrix4X4 m;
    
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            m.elements[i][j] = 0;
            for(int k = 0; k < 4; k++)
            {
                m.elements[i][j] += elements[i][k] * m2.elements[k][j];
            }
        }
    }
    
    return m;
}

Vector3 Matrix4X4::multiply_with_point3d(const Vector3& v)
{
    Vector3 result;
    
    result.x = elements[0][0]*v.x + elements[0][1]*v.y + elements[0][2]*v.z + elements[0][3]*1;
    result.y = elements[1][0]*v.x + elements[1][1]*v.y + elements[1][2]*v.z + elements[1][3]*1;
    result.z = elements[2][0]*v.x + elements[2][1]*v.y + elements[2][2]*v.z + elements[2][3]*1;

    return result;
}

Vector3 Matrix4X4::get_euler_rotation(void)
{
    double yaw, pitch, roll;
    
    if(elements[0][0] == 1 || elements[0][0] == -1)
    {
        yaw = atan2(elements[0][2], elements[2][3]);
        pitch = 0;
        roll = 0;
    }
    else
    {
        yaw = atan2(-elements[2][0], elements[0][0]);
        pitch = asin(elements[1][0]);
        roll = atan2(-elements[1][2], elements[1][1]);
    }
    return Vector3(roll, yaw, pitch);
}

void Matrix4X4::from_quaternion(const Quaternion& q)
{
    double s = q.s;
    Vector3 u = q.u;
    
    double a = 1 - (2 * u.y * u.y) - (2 * u.z * u.z);
    double b = (2 * u.x * u.y) - (2 * s * u.z);
    double c = (2 * u.x * u.z) + (2 * s * u.y);
    double d = (2 * u.x * u.y) + (2 * s * u.z);
    double e = 1 - (2 * u.x * u.x) - (2 * u.z * u.z);
    double f = (2 * u.y * u.z) - (2 * s * u.x);
    double g = (2 * u.x * u.z) - (2 * s * u.y);
    double h = (2 * u.y * u.z) + (2 * s * u.x);
    double i = 1 - (2 * u.x * u.x) - (2 * u.y * u.y);
    
    elements = {{a, b, c, 0},
                {d, e, f, 0},
                {g, h, i, 0},
                {0, 0, 0, 1}};
}

void Matrix4X4::print(void)
{
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            std::cout << elements[i][j] << ", ";
        }
        std::cout << "\n";
    }
}
