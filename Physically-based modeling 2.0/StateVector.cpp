//
//  StateVector.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "StateVector.h"

StateVector::StateVector(void)
{
    
}

StateVector::StateVector(const int& count)
{
    n = count;
    elements.reserve(2*n);
    m = new float[n];
    f = new Vector3[n];
}

StateVector::~StateVector(void)
{
    
}

StateVector StateVector::operator+(const StateVector& S) const  // vector addition
{
    StateVector S_new(n);
    
    for(int i = 0; i < 2*n; i++)
        S_new.elements[i] = elements[i] + S.elements[i];
    
    for(int i = 0; i < n; i++)
        S_new.m[i] = m[i];
    
    for(int i = 0; i < n; i++)
        S_new.f[i] = f[i];
    
    return S_new;
}

StateVector StateVector::operator-(const StateVector& S) const  // vector subtraction
{
    StateVector S_new(n);
    
    for(int i = 0; i < 2*n; i++)
        S_new.elements[i] = elements[i] - S.elements[i];
    
    for(int i = 0; i < n; i++)
        S_new.m[i] = m[i];
    
    for(int i = 0; i < n; i++)
        S_new.f[i] = f[i];
    
    return S_new;
}

StateVector StateVector::operator*(const double& scalar) const  // multiply by scalar
{
    StateVector S_new(n);
    
    for(int i = 0; i < 2*n; i++)
        S_new.elements[i] = elements[i] * scalar;
    
    for(int i = 0; i < n; i++)
        S_new.m[i] = m[i];
    
    for(int i = 0; i < n; i++)
        S_new.f[i] = f[i];
    
    return S_new;
}

StateVector StateVector::operator/(const double& scalar)             // divide by scalar
{
    StateVector S_new(n);
    
    for(int i = 0; i < 2*n; i++)
        S_new.elements[i] = elements[i] / scalar;
    
    for(int i = 0; i < n; i++)
        S_new.m[i] = m[i];
    
    for(int i = 0; i < n; i++)
        S_new.f[i] = f[i];
    
    return S_new;
}

StateVector StateVector::operator=(const StateVector& S)        // assignment
{
    for(int i = 0; i < 2*n; i++)
        elements[i] = S.elements[i];
    
    for(int i = 0; i < n; i++)
        m[i] = S.m[i];
    
    for(int i = 0; i < n; i++)
        f[i] = S.f[i];
    
    return *this;
}

void StateVector::allocate_memory(const int& count)
{
    n = count;
    elements.resize(2*n);
    m = new float[n];
    f = new Vector3[n];
}

void StateVector::print_positions(void)
{
    std::cout << "Positions:\n";
    for(int i = 0; i < n; i++)
        elements[i].print();
}

void StateVector::print_velocities(void)
{
    std::cout << "Velocities:\n";
    for(int i = 0; i < n; i++)
        elements[i+n].print();
}

void StateVector::print_masses(void)
{
    std::cout << "Masses:\n";
    for(int i = 0; i < n; i++)
        std::cout << m[i] << "\n";
}

void StateVector::print_forces(void)
{
    std::cout << "Forces:\n";
    for(int i = 0; i < n; i++)
        f[i].print();
}
