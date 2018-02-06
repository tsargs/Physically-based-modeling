//
//  StateVector.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef STATEVECTOR
#define STATEVECTOR

#include <stdio.h>
#include <vector>
#include "Vector3.h"

class StateVector
{
    public:
        std::vector<Vector3> elements;
    
        float* m;   // mass
    
        Vector3* f; // force
    
        int n;                  // number of particles or vertices
    
        StateVector(void);
        StateVector(const int& count);
        ~StateVector(void);
    
        StateVector operator+(const StateVector& S) const;  // vector addition
        StateVector operator-(const StateVector& S) const;  // vector subtraction
        StateVector operator*(const double& scalar) const;  // multiply by scalar
        StateVector operator/(const double& scalar);        // divide by scalar
        StateVector operator=(const StateVector& S);        // assignment
    
        void allocate_memory(const int& count);
        void print_positions(void);
        void print_velocities(void);
        void print_masses(void);
        void print_forces(void);
};

#endif
