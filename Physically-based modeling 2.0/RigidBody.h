//
//  RigidBody.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef RIGIDBODY
#define RIGIDBODY

#include <stdio.h>
#include <vector>
#include "Vertex.h"
#include "Strut.h"
#include "Face.h"
#include "Polygon.h"
#include "StateVectorRB.h"
#include "Wind.h"
#include "TorsionalSpring.h"

class RigidBody
{
    public:
        std::vector<Vertex> vertices;
        std::vector<Strut> struts;
        std::vector<Face> faces;
        std::vector<Polygon> polygons;
    
        StateVectorRB S;
        StateVectorRB K1;
        StateVectorRB K2;
        StateVectorRB K3;
        StateVectorRB K4;
    
        float m;            // mass of the object
        float r;            // radius of the cube along the axes from center of mass
    
        Matrix3X3 I0_inv;   // inverse of initial moment of inertia;
    
        struct force
        {
            Vector3 f;
            bool is_body_force;
            Vector3 point;
        };
        force* force_node;
        std::vector<force> forces;       // forces acting on the rigid body
    
        float cd;   // drag coefficient
        float cl;   // lift coefficient
    
        Vector3 color;
    
        bool render_wireframe;
        bool render_shaded_model;
    
        Matrix4X4 TR;    // transformation matrix
        Matrix4X4 T;     // translation matrix
        Matrix4X4 R;     // rotation matrix
    
        RigidBody();
        RigidBody(const std::string& type, const Vector3& initial_pos, const Vector3& initial_rot, const Vector3& velocity, const Vector3& angular_velocity, const float& radius, const float& mass, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type);
        ~RigidBody();
    
        void update_forces(const Vector3& g, const Vector3& w);
        StateVectorRB IntegrateEuler(StateVectorRB SV, const double& h);
        StateVectorRB IntegrateRK4(StateVectorRB SV, const double& h);
        StateVectorRB F(StateVectorRB S);

        Vector3 find_normal_and_vertex_indices(const int& e1, const int& e2, int& v1, int& v2, int& v3);
        float find_angle_and_common_vertex(const int& e1, const int& e2, int& common_vertex);
    
        void add_faces_to_polygons(void);
    
        void compute_transformation_matrix(void);
        void update_vertices(void);
};

#endif
