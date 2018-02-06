//
//  Vector2.hpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef VECTOR2
#define VECTOR2

#include <iostream>
#include <math.h>
#include "Vector3.h"

class Vector2
{
    
public:
    double x, y;
    
    //Constructors
    Vector2(void);
    Vector2(const Vector2& v);
    Vector2(const double& xVal, const double& yVal);
    
    //Destructor
    ~Vector2(void);
    
    //Member functions
    void normalize();
    Vector2 unit_vector();
    double magnitude();
    double distance(const Vector2& v);
    
    Vector2 operator+(const Vector2& v);
    Vector2 operator-(const Vector2& v);
    Vector2 operator/(const double& s);
    Vector2 operator=(const Vector2& v);
    Vector2 operator*(const double& s);
    double operator*(const Vector2& v);
    Vector3 cross(const Vector2& v);
    void print(void);
};

inline void Vector2::normalize(void)
{
    double m = magnitude();
    x /= m;
    y /= m;
}

inline Vector2 Vector2::unit_vector(void)
{
    double m = magnitude();
    return Vector2(x/m, y/m);
}

inline double Vector2::magnitude(void)
{
    return sqrt(x*x + y*y);
}

inline double Vector2::distance(const Vector2& v)
{
    return sqrt(pow(x-v.x, 2) + pow(y-v.y, 2));
}

inline Vector2 Vector2::operator+(const Vector2& v)
{
    return Vector2(x+v.x, y+v.y);
}

inline Vector2 Vector2::operator-(const Vector2& v)
{
    return Vector2(x-v.x, y-v.y);
}

inline Vector2 Vector2::operator/(const double& s)
{
    return Vector2(x/s, y/s);
}

inline Vector2 Vector2::operator=(const Vector2& v)
{
    x = v.x;
    y = v.y;
    return *this;
}

inline Vector2 Vector2::operator*(const double& s)
{
    return Vector2(x*s, y*s);
}

inline double Vector2::operator*(const Vector2& v)
{
    return x*v.x + y*v.y;
}

inline Vector3 Vector2::cross(const Vector2& v)
{
    return Vector3(0, 0, (x * v.y - v.x * y));
}

inline void Vector2::print(void)
{
    std::cout << x << ", " << y << "\n";
}

#endif
