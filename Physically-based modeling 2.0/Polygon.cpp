//
//  Polygon.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/18/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "Polygon.h"


Polygon::Polygon(void):cr(0.6), cf(0.3)
{

}

Polygon::~Polygon(void)
{

}

Vector3* Polygon::fetch_next_vertex(void)
{
    if (fetch_counter < vertices.size())
    {
        ++fetch_counter;
        return &vertices[fetch_counter-1];
    }
    else
    {
        fetch_counter = 0;
        return nullptr;
    }
}

char find_max(const double& x, const double& y, const double& z)
{
    if (x >= y)
    {
        if (x >= z)
            return 'x';
        else
            return 'z';
    }
    else
    {
        if (y >= z)
            return 'y';
        else
            return 'z';
    }
}

void Polygon::pre_compute_values(void)
{
    //Find normal
    normal = (vertices[2] - vertices[1]).cross(vertices[0] - vertices[1]);
    normal.normalize();
    
    //Find projected vertices
    max_normal_coord = find_max(abs(normal.x), abs(normal.y), abs(normal.z));
    
    for (int i = 0; i < vertices.size(); i++)
    {
        projected_vertices.push_back(get_projected_vertex(vertices[i]));
    }
    
    //Find edge vectors
    for (int i = 0; i < vertices.size(); i++)
    {
        if (i == vertices.size()-1)
            edge_vectors.push_back(projected_vertices[0]-projected_vertices[i]);
        else
            edge_vectors.push_back(projected_vertices[i+1]-projected_vertices[i]);
    }
}

Vector2 Polygon::get_projected_vertex(const Vector3& vertex)
{
    switch (max_normal_coord)
    {
        case 'x':
            return Vector2(vertex.y, vertex.z);
            
        case 'y':
            return Vector2(vertex.x, vertex.z);
            
        case 'z':
            return Vector2(vertex.x, vertex.y);
    }
    return Vector2(0,0);
}

void Polygon::prepare_matrices(const Vector2& pos_hit)
{
    for (int i = 0; i < vertices.size(); i++)
    {
        collision_check_matrices.push_back
        (Matrix2X2(edge_vectors[i].x, edge_vectors[i].y,
                  (pos_hit.x - projected_vertices[i].x), (pos_hit.y - projected_vertices[i].y)));
    }
}

bool Polygon::check_determinants(const Vector3& pos_hit_3d)
{
    Vector2 pos_hit_2d = get_projected_vertex(pos_hit_3d);
    
    prepare_matrices(pos_hit_2d);
    
    double a = collision_check_matrices[0].determinant();
    
    for (int i = 1; i < vertices.size(); i++)
    {
        double b = collision_check_matrices[i].determinant();
        if (a*b < 0)
            return false;
    }

    return true;
}
