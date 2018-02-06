//
//  RigidBodySimulator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef RIGIDBODYSIMULATOR
#define RIGIDBODYSIMULATOR

#include <stdio.h>
#include <vector>
#include "RigidBody.h"
#include "Matrix2X2.h"
#include "Vortex.h"

class RigidBodySimulator
{
    public:
        std::vector<RigidBody> rigid_bodies;
    
        double  h = (double)1/90;
        Vector3 g = Vector3(0, -10, 0);
        Vector3 wind = Vector3(0, 11, 0);  //wind forces
    
        Polygon *polygons;
        int polygon_count;
        std::vector <Wind> winds;
    
        Vortex* vortex = nullptr;
    
        //Collision handling
        double t_hit;
        Vector3 pos_hit_new;
        std::vector<Polygon*> colliding_polygons;
        Vector3 pos, pos_c, v, a, w;
        float m;
        Matrix3X3 I_inv;
        Vector3 dP, dL;
        StateVectorRB S_new;
        bool stop_edge_collision_test = false;
    
        struct EdgeCollisionInfo
        {
            int e1, e2; // edges
            int m1, m2; // mesh
            Vector3 m;  // vector from one edge to another
            bool e1_static, e2_static;
        };
    
        std::vector <EdgeCollisionInfo> edge_collision_list;
    
        struct MovingEdgeInfo
        {
            int mesh_id;
            int edge_id;
            bool is_static;
        };
        std::vector<MovingEdgeInfo> moving_edges;
        std::vector<Polygon> moving_polygons;
    
        double cr = 10;
        double cf = 0.3;
    
        //Integration method
        bool euler_integration;
        bool rk4_integration;
    
        //rigidbody generation
        Vector3 u_x, u_y, u_z;
        Matrix4X4 R;
    
        RigidBodySimulator(void);
        RigidBodySimulator(const std::string& integration_method);
        ~RigidBodySimulator(void);
    
        void update_simulation(const double& displayStep);
        StateVectorRB integrate(const int& rigid_body_id, StateVectorRB SV, const double& time_step);
        void generate_cubes(const Vector3& position, const Vector3& normal);
    
        bool check_collision(void);
        void update_collision_response(void);
    
        void process_edge_collision(const int& edge_collision_id);
    
        bool check_vertex_face_collision(void);
        void update_vertex_face_collision_response(const int& mesh_id);
    
        void prepare_moving_edge_list(void);
        void prepare_edge_collision_list(void);
        void add_cube_rigid_body(const Vector3& pos, const Vector3& _rot, const Vector3& velocity, const Vector3& angular_velocity, const float& radius, const float& mass, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type);
};

#endif
