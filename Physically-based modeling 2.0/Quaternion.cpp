//
//  Quaternion.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Quaternion.h"

Quaternion::Quaternion(void)
{

}

Quaternion::Quaternion(const double& real_part, const Vector3& imaginary_parts) : s(real_part), u(imaginary_parts)
{
    
}

Quaternion::Quaternion(const Quaternion& q) : s(q.s), u(q.u)
{
    
}

Quaternion::Quaternion(const double& pitch, const double& roll, const double& yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    
    s = cy * cr * cp + sy * sr * sp;
    u.x = cy * sr * cp - sy * cr * sp;
    u.y = cy * cr * sp + sy * sr * cp;
    u.z = sy * cr * cp - cy * sr * sp;
}

Quaternion::~Quaternion(void)
{

}

Matrix3X3 Quaternion::rotation_matrix(void)
{
    Matrix3X3 m;
    
    double a = 1 - (2 * u.y * u.y) - (2 * u.z * u.z);
    double b = (2 * u.x * u.y) - (2 * s * u.z);
    double c = (2 * u.x * u.z) + (2 * s * u.y);
    double d = (2 * u.x * u.y) + (2 * s * u.z);
    double e = 1 - (2 * u.x * u.x) - (2 * u.z * u.z);
    double f = (2 * u.y * u.z) - (2 * s * u.x);
    double g = (2 * u.x * u.z) - (2 * s * u.y);
    double h = (2 * u.y * u.z) + (2 * s * u.x);
    double i = 1 - (2 * u.x * u.x) - (2 * u.y * u.y);
    
    m.elements = {{a, b, c},
                  {d, e, f},
                  {g, h, i}};
    
    return m;
}


void Quaternion::from_euler(const double& pitch, const double& roll, const double& yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    
    s = cy * cr * cp + sy * sr * sp;
    u.x = cy * sr * cp - sy * cr * sp;
    u.y = cy * cr * sp + sy * sr * cp;
    u.z = sy * cr * cp - cy * sr * sp;
}

void Quaternion::normalize(void)
{
    double r = sqrt(s*s + u*u);
    s = s / r;
    u = u / r;
}
