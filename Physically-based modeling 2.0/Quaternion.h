//
//  Quaternion.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef QUATERNION
#define QUATERNION

#include <iostream>
#include <math.h>
#include "Matrix3X3.h"

class Quaternion
{

    public:
        double s;   // real part
        Vector3 u;  // imaginary parts
    
        //Constructors
        Quaternion(void);
        Quaternion(const Quaternion& q);
        Quaternion(const double& real_part, const Vector3& imaginary_parts);
        Quaternion(const double& pitch, const double& roll, const double& yaw);
    
        //Destructor
        ~Quaternion(void);
    
        //Member functions
        Quaternion operator=(const Quaternion& q);          // assignment
        Quaternion operator+(const Quaternion& q2) const;   // quaternion-quaternion addition
        Quaternion operator-(const Quaternion& q2) const;   // quaternion-quaternion subtraction
        Quaternion operator*(const Quaternion& q2) const;   // quaternion-quaternion multiplication
        Quaternion operator*(const double& scalar) const;   // multiply by scalar
        Quaternion operator/(const double& scalar) const;   // division by scalar
        Matrix3X3  rotation_matrix(void);                   // returns rotation matrix
        void from_euler(const double& pitch, const double& roll, const double& yaw);
        void normalize(void);
        void print (void) const;
};

inline Quaternion Quaternion::operator*(const double& scalar) const
{
    Quaternion q;
    
    q.s = s * scalar;
    q.u = u * scalar;
    
    return q;
}

inline Quaternion Quaternion::operator/(const double& scalar) const
{
    Quaternion q;
    
    q.s = s / scalar;
    q.u = u / scalar;
    
    return q;
}

inline Quaternion Quaternion::operator=(const Quaternion& q)
{
    s = q.s;
    u = q.u;
    
    return *this;
}

inline Quaternion Quaternion::operator+(const Quaternion& q2) const
{
    return Quaternion(s+q2.s, u+q2.u);
}

inline Quaternion Quaternion::operator-(const Quaternion& q2) const
{
    return Quaternion(s-q2.s, u-q2.u);
}

inline Quaternion Quaternion::operator*(const Quaternion& q2) const
{
    Quaternion q;
    
    q.s = (s * q2.s - u * q2.u);
    q.u = (q2.u * s) + (u * q2.s) + u.cross(q2.u);
    
    return q;
}

inline void Quaternion::print(void) const
{
    std::cout << s << ", " << u.x << ", "<< u.y << ", "<< u.z << "\n";
}

#endif
