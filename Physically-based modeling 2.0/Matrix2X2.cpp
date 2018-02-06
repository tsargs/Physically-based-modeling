//
//  Matrix2X2.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Matrix2X2.h"

Matrix2X2::Matrix2X2(void): a11(0), a12(0), a21(0), a22(0)
{

}

Matrix2X2::Matrix2X2(const Matrix2X2& m): a11(m.a11), a12(m.a12), a21(m.a21), a22(m.a22)
{

}

Matrix2X2::Matrix2X2(const double& b11, const double& b12, const double& b21, const double& b22): a11(b11), a12(b12), a21(b21), a22(b22)
{

}

Matrix2X2::~Matrix2X2(void)
{

}