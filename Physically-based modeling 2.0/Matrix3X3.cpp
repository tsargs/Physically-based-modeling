//
//  Matrix3X3.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Matrix3X3.h"

Matrix3X3::Matrix3X3(void)
{
    elements.resize(3);
    for (int i = 0; i < 3; i++)
        elements[i].resize(3);
}

Matrix3X3::~Matrix3X3()
{
    
}

void Matrix3X3::set_rotation_x(const float& theta)
{
    elements = {{1,0,0},
                {0,cos(theta),-sin(theta)},
                {0,sin(theta),cos(theta)}};
}

void Matrix3X3::add_vector3_as_column(const Vector3& v, const int& column)
{
    elements[0][column] = v.x;
    elements[1][column] = v.y;
    elements[2][column] = v.z;
}

Vector3 Matrix3X3::operator*(const Vector3& v)
{
    Vector3 result;
    
    double a = elements[0][0];
    double b = elements[0][1];
    double c = elements[0][2];
    
    double d = elements[1][0];
    double e = elements[1][1];
    double f = elements[1][2];
    
    double g = elements[2][0];
    double h = elements[2][1];
    double i = elements[2][2];
    
    result.x = a*v.x + b*v.y + c*v.z;
    result.y = d*v.x + e*v.y + f*v.z;
    result.z = g*v.x + h*v.y + i*v.z;
    
    return result;
}

Matrix3X3 Matrix3X3::operator*(const Matrix3X3& m2)
{
    Matrix3X3 m;
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            m.elements[i][j] = 0;
            for(int k = 0; k < 3; k++)
            {
                m.elements[i][j] += elements[i][k] * m2.elements[k][j];
            }
        }
    }
    
    return m;
}

void Matrix3X3::print(void)
{
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            std::cout << elements[i][j] << ", ";
        }
        std::cout << "\n";
    }
}

Matrix3X3 Matrix3X3::inverse_matrix(void)
{
    Matrix3X3 m_inv;
    
    double a = elements[0][0];
    double b = elements[0][1];
    double c = elements[0][2];
    
    double d = elements[1][0];
    double e = elements[1][1];
    double f = elements[1][2];
    
    double g = elements[2][0];
    double h = elements[2][1];
    double i = elements[2][2];
    
    double determinant = (a*e*i) + (b*f*g) + (c*d*h)- ((c*e*g) + (b*d*i) + (a*f*h));
    
    m_inv.elements = {{e*i - f*h, c*h - b*i, b*f - c*e},
                      {f*g - d*i, a*i - c*g, c*d - a*f},
                      {d*h - e*g, b*g - a*h, a*e - b*d}};
    
    return m_inv*(1/determinant);
}

Matrix3X3 Matrix3X3::transpose(void)
{
    Matrix3X3 m_transpose;
    
    double a = elements[0][0];
    double b = elements[0][1];
    double c = elements[0][2];
    
    double d = elements[1][0];
    double e = elements[1][1];
    double f = elements[1][2];
    
    double g = elements[2][0];
    double h = elements[2][1];
    double i = elements[2][2];
    
    m_transpose.elements = {{a, d, g},
                            {b, e, h},
                            {c, f, i}};
    
    return m_transpose;
}

Matrix3X3 Matrix3X3::operator*(const double& s) const
{
    Matrix3X3 m;
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            m.elements[i][j] = elements[i][j] * s;
        }
    }
    
    return m;
}

Matrix3X3 Matrix3X3::operator=(const Matrix3X3& m2)
{
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            elements[i][j] = m2.elements[i][j];
        }
    }
    return *this;
}

Vector3 Matrix3X3::get_euler_rotation(void)
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
