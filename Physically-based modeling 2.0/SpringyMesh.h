//
//  SpringyMesh.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef SPRINGYMESH
#define SPRINGYMESH

#include <stdio.h>
#include <vector>
#include "Vertex.h"
#include "Strut.h"
#include "Face.h"
#include "Polygon.h"
#include "StateVector.h"
#include "Wind.h"
#include "TorsionalSpring.h"

class SpringyMesh
{
    public:
        std::vector<Vertex> vertices;
        std::vector<Strut> struts;
        std::vector<Face> faces;
        std::vector<Polygon> polygons;
        std::vector<TorsionalSpring> torsional_springs;
    
        StateVector S;
        StateVector K1;
        StateVector K2;
        StateVector K3;
        StateVector K4;
    
        float k;    // spring stiffness of each strut
        float d;    // damping constant of each strut
    
        float m;    // mass of the object
    
        float T;    // time constant
        float P;    // period of oscillation
    
        float cd;   // drag coefficient
        float cl;   // lift coefficient
    
        Vector3 color;
    
        bool render_wireframe;
        bool render_shaded_model;
    
        SpringyMesh();
        SpringyMesh(const std::string& type, const Vector3& initial_pos, const Vector3& velocity, const float& radius, const float& mass, const float& time_constant, const float& period_of_oscillation, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type);
        ~SpringyMesh();
    
        void update_forces(const Vector3& g, const Vector3& w);
        Vector3 find_normal_and_vertex_indices(const int& e1, const int& e2, int& v1, int& v2, int& v3);
        float find_angle_and_common_vertex(const int& e1, const int& e2, int& common_vertex);
        void add_torsional_spring(const int& f1, const int& f2, const int& common_strut);
        void add_faces_to_polygons(void);
};

#endif
