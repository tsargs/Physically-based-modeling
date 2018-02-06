//
//  StateVectorRB.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "StateVectorRB.h"

StateVectorRB::StateVectorRB(void)
{
    
}

StateVectorRB::~StateVectorRB(void)
{
    
}

StateVectorRB StateVectorRB::operator+(const StateVectorRB& S) const  // vector addition
{
    StateVectorRB S_new;
    
    S_new.pos = pos + S.pos;
    S_new.q   = q + S.q;
    S_new.P   = P + S.P;
    S_new.L   = L + S.L;
    
    return S_new;
}

StateVectorRB StateVectorRB::operator-(const StateVectorRB& S) const  // vector subtraction
{
    StateVectorRB S_new;
    
    S_new.pos = pos - S.pos;
    S_new.q   = q - S.q;
    S_new.P   = P - S.P;
    S_new.L   = L - S.L;
    
    return S_new;
}

StateVectorRB StateVectorRB::operator*(const double& scalar) const  // multiply by scalar
{
    StateVectorRB S_new;
    
    S_new.pos = pos * scalar;
    S_new.q   = q * scalar;
    S_new.P   = P * scalar;
    S_new.L   = L * scalar;
    
    return S_new;
}

StateVectorRB StateVectorRB::operator/(const double& scalar)             // divide by scalar
{
    StateVectorRB S_new;
    
    S_new.pos = pos / scalar;
    S_new.q   = q * (1/scalar);
    S_new.P   = P / scalar;
    S_new.L   = L / scalar;
    
    return S_new;
}

StateVectorRB StateVectorRB::operator=(const StateVectorRB& S)        // assignment
{
    pos = S.pos;
    q   = S.q;
    P   = S.P;
    L   = S.L;
    
    return *this;
}
