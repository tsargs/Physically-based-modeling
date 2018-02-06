//
//  SpringyMeshSimulator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SpringyMeshSimulator.h"

SpringyMeshSimulator::SpringyMeshSimulator(void)
{
    
}

SpringyMeshSimulator::SpringyMeshSimulator(const std::string& integration_method)
{
    if(integration_method.compare("euler") == 0)
        euler_integration = true;
    else
        rk4_integration = true;
}

SpringyMeshSimulator::~SpringyMeshSimulator(void)
{
    
}

void SpringyMeshSimulator::update_simulation(const double& displayStep)
{
    double t = 0;
    
    while (t < displayStep)
    {
        for (int i = 0 ; i < meshes.size(); i++)
        {
            meshes[i].update_forces(g, w);
            
            if(euler_integration)
                meshes[i].S = IntegrateEuler(meshes[i].S);
            else
                meshes[i].S = IntegrateRK4(meshes[i].S, meshes[i].K1, meshes[i].K2, meshes[i].K3, meshes[i].K4);
           
            // vertex-static face collision test
            for(int j = 0; j < meshes[i].S.n; j++)
            {
                t_hit = h;
                pos = meshes[i].S.elements[j];
                v = meshes[i].S.elements[j + meshes[i].S.n];
                
                bool collision = check_collision();
                if (collision)
                {
                    update_collision_response();
                    meshes[i].S.elements[j] = pos;
                    meshes[i].S.elements[j + meshes[i].S.n] = v;
                }
            }
        }
        
        // vertex-face collision
        for(int j = 0; j < meshes.size(); j++)
        {
            for(int k = 0; k < meshes.size(); k++)
            {
                if(j != k)
                {
                    for(int m = 0; m < meshes[k].polygons.size(); m++)
                    {
                        Vector3 pos1 = meshes[k].S.elements[meshes[k].polygons[m].v1];
                        Vector3 pos2 = meshes[k].S.elements[meshes[k].polygons[m].v2];
                        Vector3 pos3 = meshes[k].S.elements[meshes[k].polygons[m].v3];
                        Polygon polygon;
                        polygon.add_vertex(pos1.x, pos1.y, pos1.z);
                        polygon.add_vertex(pos2.x, pos2.y, pos2.z);
                        polygon.add_vertex(pos3.x, pos3.y, pos3.z);
                        polygon.v1 = meshes[k].polygons[m].v1;
                        polygon.v2 = meshes[k].polygons[m].v2;
                        polygon.v3 = meshes[k].polygons[m].v3;
                        polygon.pre_compute_values();
                        moving_polygons.push_back(polygon);
                    }
                    
                    for(int l = 0; l < meshes[j].S.n; l++)
                    {
                        pos = meshes[j].S.elements[l];
                        v = meshes[j].S.elements[l + meshes[j].S.n];
                        
                        t_hit = h;
                        bool collision = check_vertex_face_collision();
                        
                        if (collision)
                        {
                            update_vertex_face_collision_response(k);
                            meshes[j].S.elements[l] = pos;
                            meshes[j].S.elements[l + meshes[j].S.n] = v;
                        }
                    }
                    
                    moving_polygons.clear();
                }
            }
        }
        
        // edge-edge collision
        for(int j = 0; j < edge_collision_list.size(); j++)
        {
            process_edge_collision(j);
        }
        t += h;
    }
}

bool SpringyMeshSimulator::check_vertex_face_collision(void)
{
    double t_hit_tol = 0; //0.00008;
    bool collision = false;
    
    for (int i = 0; i < moving_polygons.size(); i++)
    {
        Vector3 p = moving_polygons[i].vertices[0];
        Vector3 n = moving_polygons[i].normal;
        
        //Check collision with the plane that has the polygon
        double t_hit_new = ((p - pos)*n)/(v*n);
        
        if (t_hit_new < t_hit && t_hit_new >= t_hit_tol)
        {
            //Find the collision point
            Vector3 pos_hit_new = pos + (v * t_hit_new);
            
            //Project vertices to 2D plane
            //Calculate 2D edge vectors for each projected vertex
            //Form 2X2 matrix for each vertex with edge vector and vector toward x_hit_new
            //Find the determinant of the matrix for each row
            bool same_sign = moving_polygons[i].check_determinants(pos_hit_new);
            
            if (same_sign)
            {
                if (t_hit_new < t_hit)
                {
                    colliding_polygons.clear();
                    t_hit = t_hit_new;
                    colliding_polygons.push_back(&moving_polygons[i]);
                }
                collision = true;
            }
        }
    }
    return collision;
}

void SpringyMeshSimulator::update_vertex_face_collision_response(const int& mesh_id)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        Vector3 p = colliding_polygons[i]->vertices[0];
        Vector3 n = colliding_polygons[i]->normal;
        double cr = colliding_polygons[i]->cr;
        double cf = colliding_polygons[i]->cf;
        
        double dist = (pos - p)*n;
        
        pos = pos - n * (1 + cr) * dist;
        Vector3 v_normal = n*(v*n);
        Vector3 v_tangential = v - v_normal;
        
        v = v_normal*(-cr) + v_tangential*(1-cf);
        
        meshes[mesh_id].S.elements[colliding_polygons[i]->v1 + meshes[mesh_id].S.n] = -v;
        meshes[mesh_id].S.elements[colliding_polygons[i]->v2 + meshes[mesh_id].S.n] = -v;
        meshes[mesh_id].S.elements[colliding_polygons[i]->v3 + meshes[mesh_id].S.n] = -v;
    }
    colliding_polygons.clear();
}


void SpringyMeshSimulator::process_edge_collision(const int& edge_collision_id)
{
    Vector3 p1, p2, q1, q2;
    Vector3 a, b, a_hat, b_hat;
    Vector3 n;
    Vector3 r;
    Vector3 m;
    
    float s, t;
    
    int m1 = edge_collision_list[edge_collision_id].m1; //mesh one
    int e1 = edge_collision_list[edge_collision_id].e1; //edge one
    
    int p1_index = meshes[m1].struts[e1].v1;
    int p2_index = meshes[m1].struts[e1].v2;
    
    p1 = meshes[m1].S.elements[p1_index];
    p2 = meshes[m1].S.elements[p2_index];
    
    int m2 = edge_collision_list[edge_collision_id].m2; //mesh two
    int e2 = edge_collision_list[edge_collision_id].e2; //edge two
    
    int q1_index = meshes[m2].struts[e2].v1;
    int q2_index = meshes[m2].struts[e2].v2;
    
    q1 = meshes[m2].S.elements[q1_index];
    q2 = meshes[m2].S.elements[q2_index];
    
    a = p2-p1;
    b = q2-q1;
    
    a_hat = a.unit_vector();
    b_hat = b.unit_vector();
    
    n = (a.cross(b)).unit_vector();
    
    if (n.x == 0 && n.y == 0 && n.z == 0)
        return; // zero vector (a and b are parallel)
    
    r = q1-p1;
    
    Vector3 a_hat_cross_n = a_hat.cross(n);
    Vector3 b_hat_cross_n = b_hat.cross(n);
    
    s = (r * b_hat_cross_n)/(a * b_hat_cross_n);
   
    if(s < 0 || s > 1)
        return;

    t = (-r * a_hat_cross_n)/(b * a_hat_cross_n);
    
    if(t < 0 || t > 1)
        return;
    
    Vector3 pa = p1 + a*s;
    Vector3 qa = q1 + b*t;
    
    m = qa - pa;
    
    Vector3 m_prev = edge_collision_list[edge_collision_id].m;
    edge_collision_list[edge_collision_id].m = m;
    
   
    if (m.magnitude() > 0.02)
        return;
    else
    {
        if (m_prev * m >= 0)
            return;
    }
    
    // collision detected at this point
    float up, vp;
    float uq, vq;
    
    vp = s;
    up = 1 - vp;
    
    vq = t;
    uq = 1 - vq;
    
    // mass of the vertices
    float mp1, mp2;
    float mq1, mq2;
    
    mp1 = meshes[m1].S.m[p1_index];
    mp2 = meshes[m1].S.m[p2_index];
    
    mq1 = meshes[m2].S.m[q1_index];
    mq2 = meshes[m2].S.m[q2_index];
    
    // effective mass at collision point
    float mp, mq;
    
    mp = (up * mp1) + (vp * mp2)/(up*up + vp*vp);
    mq = (uq * mq1) + (vq * mq2)/(uq*uq + vq*vq);
    
    //velocity of the vertices
    Vector3 vp1, vp2;
    Vector3 vq1, vq2;
    Vector3 vel_p, vel_q;
    
    vp1 = meshes[m1].S.elements[p1_index + meshes[m1].S.n];
    vp2 = meshes[m1].S.elements[p2_index + meshes[m1].S.n];
    
    vq1 = meshes[m2].S.elements[q1_index + meshes[m2].S.n];
    vq2 = meshes[m2].S.elements[q2_index + meshes[m2].S.n];
    
    vel_p = (vp1 * up) + (vp2 * vp);
    vel_q = (vq1 * uq) + (vq2 * vq);
    
    // center of momentum
    Vector3 cm = (vel_p * mp) + (vel_q * mq)/(mp + mq);
    
    // velocities relative to center of momentum
    Vector3 vel_p_cm = vel_p - cm;
    Vector3 vel_q_cm = vel_q - cm;
    
    //normal direction
    Vector3 vel_p_cm_n = n*(vel_p_cm*n);
    Vector3 vel_q_cm_n = n*(vel_q_cm*n);
    
    vel_p_cm_n = vel_p_cm_n * (-cr);
    vel_q_cm_n = vel_q_cm_n * (-cr);
    
    // tangential direction
    Vector3 vel_p_cm_t = vel_p_cm - vel_p_cm_n;
    Vector3 vel_q_cm_t = vel_q_cm - vel_q_cm_n;
    
    vel_p_cm_t = vel_p_cm_t * (1-cf);
    vel_q_cm_t = vel_q_cm_t * (1-cf);
    
    // total velocity
    vel_p_cm = vel_p_cm_n + vel_p_cm_t;
    vel_q_cm = vel_q_cm_n + vel_q_cm_t;
    
    Vector3 vel_p_new = vel_p_cm + cm;
    Vector3 vel_q_new = vel_q_cm + cm;
    
    Vector3 vel_p_delta = vel_p_new - vel_p;
    Vector3 vel_q_delta = vel_q_new - vel_q;
    
    Vector3 vel_p_delta_prime = vel_p_delta /(up*up + vp*vp);
    Vector3 vel_q_delta_prime = vel_q_delta /(uq*uq + vq*vq);
    
    Vector3 vel_p1, vel_p2;
    Vector3 vel_q1, vel_q2;
    
    vel_p1 = vp1 + (vel_p_delta_prime * up);
    vel_p2 = vp2 + (vel_p_delta_prime * vp);
    
    vel_q1 = vq1 + (vel_q_delta_prime * uq);
    vel_q2 = vq2 + (vel_q_delta_prime * vq);
    
    meshes[m1].S.elements[p1_index + meshes[m1].S.n] = vel_p1;
    meshes[m1].S.elements[p2_index + meshes[m1].S.n] = vel_p2;
    
    meshes[m2].S.elements[q1_index + meshes[m2].S.n] = vel_q1;
    meshes[m2].S.elements[q2_index + meshes[m2].S.n] = vel_q2;
 
    /*
    meshes[m1].S.elements[p1_index + meshes[m1].S.n] = vel_p_new;
    meshes[m1].S.elements[p2_index + meshes[m1].S.n] = vel_p_new;
    
    meshes[m2].S.elements[q1_index + meshes[m2].S.n] = vel_q_new;
    meshes[m2].S.elements[q2_index + meshes[m2].S.n] = vel_q_new;
     */
}

StateVector SpringyMeshSimulator::IntegrateEuler(StateVector S)
{
    return S + F(S) * h;
}

StateVector SpringyMeshSimulator::IntegrateRK4(StateVector S, StateVector K1, StateVector K2, StateVector K3, StateVector K4)
{
    K1 = F(S);
    K2 = F(S + K1*(h*0.5));
    K3 = F(S + K2*(h*0.5));
    K4 = F(S + K3*h);
    
    return S + (K1 + (K2*2) + (K3*2) + K4)*(double)(h/6);
}

StateVector SpringyMeshSimulator::F(StateVector S)
{
    StateVector S_dr = Accelerations(S);
    
    for(int i = 0; i < S.n; i++)
    {
        S_dr.elements[i] = S.elements[i + S.n];
    }
    
    return S_dr;
}

StateVector SpringyMeshSimulator::Accelerations(StateVector S)
{
    StateVector S_dr(S.n);
    
    for(int i = 0; i < S.n; i++)
    {
        float m = S.m[i];
        Vector3 f = S.f[i];
        
        S_dr.elements[i+S.n] = f * (1/m);   // g - v * (d/m);
    }
    
    return S_dr;
}

bool SpringyMeshSimulator::check_collision(void)
{
    double t_hit_tol = 0; //0.00008;
    bool collision = false;
    
    for (int i = 0; i < polygon_count; i++)
    {
        Vector3 p = polygons[i].vertices[0];
        Vector3 n = polygons[i].normal;
        
        //Check collision with the plane that has the polygon
        double t_hit_new = ((p - pos)*n)/(v*n);
       
        if (t_hit_new < t_hit && t_hit_new >= t_hit_tol)
        {
            //Find the collision point
            Vector3 pos_hit_new = pos + (v * t_hit_new);
            
            //Project vertices to 2D plane
            //Calculate 2D edge vectors for each projected vertex
            //Form 2X2 matrix for each vertex with edge vector and vector toward x_hit_new
            //Find the determinant of the matrix for each row
            bool same_sign = polygons[i].check_determinants(pos_hit_new);
            
            if (same_sign)
            {
                if (t_hit_new < t_hit)
                {
                    colliding_polygons.clear();
                    t_hit = t_hit_new;
                    colliding_polygons.push_back(&polygons[i]);
                }
                collision = true;
            }
        }
    }
    return collision;
}

void SpringyMeshSimulator::update_collision_response(void)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        Vector3 p = colliding_polygons[i]->vertices[0];
        Vector3 n = colliding_polygons[i]->normal;
        double cr = colliding_polygons[i]->cr;
        double cf = colliding_polygons[i]->cf;
        
        double dist = (pos - p)*n;
        
        pos = pos - n * (1 + cr) * dist;
        Vector3 v_normal = n*(v*n);
        Vector3 v_tangential = v - v_normal;
        
        v = v_normal*(-cr) + v_tangential*(1-cf);
    }
    colliding_polygons.clear();
}

void SpringyMeshSimulator::add_cube_springy_mesh(const Vector3& pos, const Vector3& velocity, const float& radius, const float& mass, const float& time_constant, const float& period_of_oscillation, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type)
{
    meshes.push_back(SpringyMesh("cube", pos, velocity, radius, mass, time_constant, period_of_oscillation, drag_coefficient, lift_coefficient, mesh_color, render_type));
}

void SpringyMeshSimulator::prepare_moving_edge_list(void)
{
    for(int i = 0; i < meshes.size(); i++)
    {
        for(int j = 0; j < meshes[i].struts.size(); j++)
        {
            MovingEdgeInfo moving_edge_info;
            moving_edge_info.mesh_id = i;
            moving_edge_info.edge_id = j;
            moving_edge_info.is_static = false;
            moving_edges.push_back(moving_edge_info);
        }
    }
}

void SpringyMeshSimulator::prepare_edge_collision_list(void)
{
    prepare_moving_edge_list();
    for(int i = 0; i < moving_edges.size(); i++)
    {
        for(int j = 0; j < moving_edges.size(); j++)
        {
            if(moving_edges[i].mesh_id != moving_edges[j].mesh_id)
            {
                EdgeCollisionInfo collision_info;
                collision_info.e1        = moving_edges[i].edge_id;
                collision_info.m1        = moving_edges[i].mesh_id;
                collision_info.e1_static = moving_edges[i].is_static;
                
                collision_info.e2        = moving_edges[j].edge_id;
                collision_info.m2        = moving_edges[j].mesh_id;
                collision_info.e2_static = moving_edges[j].is_static;
                
                edge_collision_list.push_back(collision_info);
            }
        }
    }
}
