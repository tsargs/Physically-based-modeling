//
//  SpringyMeshSimulator.h
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#ifndef SPRINGYMESHSIMULATOR
#define SPRINGYMESHSIMULATOR

#include <stdio.h>
#include <vector>
#include "SpringyMesh.h"
#include "Matrix2X2.h"
#include "Vortex.h"

class SpringyMeshSimulator
{
    public:
        std::vector<SpringyMesh> meshes;
    
        double  h = (double)1/90;
        Vector3 g = Vector3(0, -10, 0);
        Vector3 w = Vector3(0, 16, 0);  //wind forces
    
        Polygon *polygons;
        int polygon_count;
        std::vector <Wind> winds;
    
        Vortex* vortex = nullptr;
    
        //Collision handling
        double t_hit;
        std::vector<Polygon*> colliding_polygons;
        Vector3 pos, v, a;
    
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
    
        SpringyMeshSimulator(void);
        SpringyMeshSimulator(const std::string& integration_method);
        ~SpringyMeshSimulator(void);
    
        void update_simulation(const double& displayStep);
        StateVector IntegrateEuler(StateVector S);
        StateVector IntegrateRK4(StateVector S, StateVector K1, StateVector K2, StateVector K3, StateVector K4);
        StateVector F(StateVector S);
        StateVector Accelerations(StateVector S);
    
        bool check_collision(void);
        void update_collision_response(void);
    
        void process_edge_collision(const int& edge_collision_id);
    
        bool check_vertex_face_collision(void);
        void update_vertex_face_collision_response(const int& mesh_id);
    
        void prepare_moving_edge_list(void);
        void prepare_edge_collision_list(void);
        void add_cube_springy_mesh(const Vector3& pos, const Vector3& velocity, const float& radius, const float& mass, const float& time_constant, const float& period_of_oscillation, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type);
};

#endif
