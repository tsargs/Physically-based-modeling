//
//  Vector3.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/14/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef VECTOR3
#define VECTOR3

#include <iostream>
#include <math.h>

class Vector3
{

    public:
        double x, y, z;
    
        //Constructors
        Vector3(void);
        Vector3(const Vector3& v);
        Vector3(const double& xVal, const double& yVal, const double& zVal);
    
        //Destructor
        ~Vector3(void);
    
        //Member functions
    
        void normalize();
        Vector3 unit_vector() const;
        double magnitude() const;
        double distance(const Vector3& v);
        double angle(const Vector3& v);
    
        Vector3 operator+(const Vector3& v) const;  // vector addition
        Vector3 operator-(const Vector3& v) const;  // vector subtraction
        Vector3 operator*(const double& s) const;   // multiply by scalar
        Vector3 operator/(const double& s) const;   // divide by scalar
        Vector3 operator-(void) const;              // unary negation
        Vector3 operator=(const Vector3& v);        // assignment
        double  operator*(const Vector3& v) const;  // dot product
        Vector3 cross(const Vector3& v) const;      // cross product
        void print (void) const;
};

inline void Vector3::normalize(void)
{
    double m = magnitude();
    x /= m;
    y /= m;
    z /= m;
}

inline Vector3 Vector3::unit_vector(void) const
{
    double m = magnitude();
    
    if (m == 0)
        m = 1; // to avoid divide by zero condition
    
    return Vector3(x/m, y/m, z/m);
}

inline double Vector3::magnitude(void) const
{
    return sqrt(x*x + y*y + z*z);
}

inline double Vector3::distance(const Vector3& v)
{
    return sqrt(pow(x-v.x, 2) + pow(y-v.y, 2) + pow(z-v.z, 2));
}

inline double Vector3::angle(const Vector3& v)
{
    Vector3 this_vector(x,y,z);
    return acos(this_vector*v/(this_vector.magnitude()*v.magnitude()));
}

inline Vector3 Vector3::operator+(const Vector3& v) const
{
    return Vector3(x+v.x, y+v.y, z+v.z);
}

inline Vector3 Vector3::operator-(const Vector3& v) const
{
    return Vector3(x-v.x, y-v.y, z-v.z);
}

inline Vector3 Vector3::operator*(const double& s) const
{
    return Vector3(x*s, y*s, z*s);
}

inline Vector3 Vector3::operator/(const double& s) const
{
    return Vector3(x/s, y/s, z/s);
}

inline Vector3 Vector3::operator-(void) const
{
    return Vector3(-x, -y, -z);
}

inline Vector3 Vector3::operator=(const Vector3& v)
{
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
}

inline double Vector3::operator*(const Vector3& v) const
{
    return x*v.x + y*v.y + z*v.z;
}

inline Vector3 Vector3::cross(const Vector3& v) const
{
    return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
}

inline void Vector3::print(void) const
{
    std::cout << x << ", " << y << ", "<< z << "\n";
}

#endif
