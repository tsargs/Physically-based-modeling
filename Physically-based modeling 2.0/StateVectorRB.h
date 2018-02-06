//
//  StateVectorRB.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef STATEVECTORRB
#define STATEVECTORRB

#include <stdio.h>
#include <vector>
#include "Quaternion.h"
#include "Matrix4X4.h"

class StateVectorRB
{
    public:
        Vector3 pos;        // position
        Quaternion q;       // rotation (quaternion)
        Vector3 P;          // linear momentum
        Vector3 L;          // angular momentum

        /*
        Vector3 v;          // linear velocity
        Vector3 w;          // angular velocity
         */
    
        Vector3 c;          // center of mass
    
        StateVectorRB(void);
        ~StateVectorRB(void);
    
        StateVectorRB operator+(const StateVectorRB& S) const;  // vector addition
        StateVectorRB operator-(const StateVectorRB& S) const;  // vector subtraction
        StateVectorRB operator*(const double& scalar) const;    // multiply by scalar
        StateVectorRB operator/(const double& scalar);          // divide by scalar
        StateVectorRB operator=(const StateVectorRB& S);        // assignment
};

#endif
