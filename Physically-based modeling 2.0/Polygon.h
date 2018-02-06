//
//  Polygon.hpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef POLYGON
#define POLYGON

#include <vector>
#include "Vector3.h"
#include "Vector2.h"
#include "Matrix2X2.h"

class Polygon
{
    public:
        std::vector<Vector3> vertices;
        std::vector<Vector2> projected_vertices;
        std::vector<Vector2> edge_vectors;
        std::vector<Matrix2X2> collision_check_matrices;
        Vector3 normal;
        Vector3 color;
    
        int v1, v2, v3;
    
        //  coefficient of restitution (fraction of speed with which the object will move away from plane)
        //  0 => inelastic collision; most of the momentum in the normal direction is lost during collision
        //  1 => equal energy added to the colliding object in the normal direction
        double cr;
    
        //***  coefficient of friction (fraction of tangential speed lost during collision)
        //  0 => no friction; very slippery
        //  1 => tangential velocity is completely wiped out; very rough surface
        //***
        double cf;
    
        double dist;  //distance for resting condition check
    
        int fetch_counter = 0;
        char max_normal_coord;
    
        Polygon(void);
    
        ~Polygon(void);
    
        Polygon operator=(const Polygon& p);
    
        void add_vertex(const double& x, const double& y, const double& z);
        void pre_compute_values(void);
        Vector3* fetch_next_vertex(void);
        Vector2 get_projected_vertex(const Vector3& vertex);
        void prepare_matrices(const Vector2& x_hit);
        bool check_determinants(const Vector3& x_hit);
};

inline Polygon Polygon::operator=(const Polygon& p)
{
    for(int i=0; i<p.vertices.size(); i++)
    {
        vertices.push_back(p.vertices[i]);
    }
    
    normal = p.normal;
    color = p.color;

    return *this;
}

inline void Polygon::add_vertex(const double& x, const double& y, const double& z)
{
    vertices.push_back(Vector3(x, y, z));
}

#endif
