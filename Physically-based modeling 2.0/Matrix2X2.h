//
//  Matrix2X2.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef MATRIX2X2
#define MATRIX2X2

#include <iostream>
#include <math.h>

class Matrix2X2
{
    
public:
    double a11, a12;
    double a21, a22;
    
    //Constructors
    Matrix2X2(void);
    Matrix2X2(const Matrix2X2& m);
    Matrix2X2(const double& b11, const double& b12, const double& b21, const double& b22);
    
    //Destructor
    ~Matrix2X2(void);
    
    //Member functions
    double determinant();
    Matrix2X2 operator=(const Matrix2X2& m);
    void print(void);
};

inline double Matrix2X2::determinant()
{
    return (a11*a22) - (a21*a12);
}

inline Matrix2X2 Matrix2X2::operator=(const Matrix2X2& m)
{
    a11 = m.a11;
    a12 = m.a12;
    a21 = m.a21;
    a22 = m.a22;
    return *this;
}

inline void Matrix2X2::print(void)
{
    std::cout << a11 << ", " << a12 << "\n";
    std::cout << a21 << ", " << a22 << "\n";
}

#endif

