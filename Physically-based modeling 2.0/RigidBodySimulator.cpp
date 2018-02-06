//
//  RigidBodySimulator.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "RigidBodySimulator.h"

RigidBodySimulator::RigidBodySimulator(void)
{
    
}

RigidBodySimulator::RigidBodySimulator(const std::string& integration_method)
{
    if(integration_method.compare("euler") == 0)
        euler_integration = true;
    else
        rk4_integration = true;
}

RigidBodySimulator::~RigidBodySimulator(void)
{
    
}

void RigidBodySimulator::update_simulation(const double& displayStep)
{
    double t = 0;
    
    while (t < displayStep && h > 0)
    {
        for (int i = 0 ; i < rigid_bodies.size(); i++)
        {
            rigid_bodies[i].update_forces(g, wind);
            
            //S_new = integrate(i, rigid_bodies[i].S, h);
            
            //rigid_bodies[i].compute_transformation_matrix();
            //rigid_bodies[i].update_vertices();  // compute world co-ordinates
           
            // vertex-static face collision test
            for(int j = 0; j < rigid_bodies[i].vertices.size(); j++)
            {
                t_hit = h;
                pos = rigid_bodies[i].vertices[j].pos;
                m = rigid_bodies[i].m;
                v = rigid_bodies[i].S.P/m;
                pos_c = rigid_bodies[i].S.pos;  // center of mass
                Matrix3X3 R = rigid_bodies[i].S.q.rotation_matrix();
                I_inv = R * rigid_bodies[i].I0_inv * R.transpose();    // inverse of moment of inertia
                w =  I_inv * rigid_bodies[i].S.L;
                
                bool collision = check_collision();
                
                if (collision)
                {
                    dP = Vector3(0,0,0);
                    dL = Vector3(0,0,0);
                    update_collision_response();
                    
                    rigid_bodies[i].S.P = rigid_bodies[i].S.P + (dP * 1.5);
                    rigid_bodies[i].S.L = rigid_bodies[i].S.L + dL;
                }
            }
            
            //rigid_bodies[i].S = S_new;
            
            rigid_bodies[i].S = integrate(i, rigid_bodies[i].S, h);
            
            rigid_bodies[i].compute_transformation_matrix();
            rigid_bodies[i].update_vertices();  // compute world co-ordinates
        }
        
        // vertex-face collision
        for(int j = 0; j < rigid_bodies.size(); j++)
        {
            for(int k = 0; k < rigid_bodies.size(); k++)
            {
                if(j != k)
                {
                    for(int m = 0; m < rigid_bodies[k].polygons.size(); m++)
                    {
                        Vector3 pos1 = rigid_bodies[k].vertices[rigid_bodies[k].polygons[m].v1].pos;
                        Vector3 pos2 = rigid_bodies[k].vertices[rigid_bodies[k].polygons[m].v2].pos;
                        Vector3 pos3 = rigid_bodies[k].vertices[rigid_bodies[k].polygons[m].v3].pos;
                        Polygon polygon;
                        polygon.add_vertex(pos1.x, pos1.y, pos1.z);
                        polygon.add_vertex(pos2.x, pos2.y, pos2.z);
                        polygon.add_vertex(pos3.x, pos3.y, pos3.z);
                        polygon.v1 = rigid_bodies[k].polygons[m].v1;
                        polygon.v2 = rigid_bodies[k].polygons[m].v2;
                        polygon.v3 = rigid_bodies[k].polygons[m].v3;
                        polygon.pre_compute_values();
                        moving_polygons.push_back(polygon);
                    }
                    
                    for(int l = 0; l < rigid_bodies[j].vertices.size(); l++)
                    {
                        t_hit = h;
                        pos = rigid_bodies[j].vertices[l].pos;
                        m = rigid_bodies[j].m;
                        v = rigid_bodies[j].S.P/m;
                        pos_c = rigid_bodies[j].S.pos;  // center of mass
                        Matrix3X3 R = rigid_bodies[j].S.q.rotation_matrix();
                        I_inv = R * rigid_bodies[j].I0_inv * R.transpose();    // inverse of moment of inertia
                        w =  I_inv * rigid_bodies[j].S.L;
                        
                        bool collision = check_vertex_face_collision();
                        
                        if (collision)
                        {
                            dP = Vector3(0,0,0);
                            dL = Vector3(0,0,0);
                            
                            update_vertex_face_collision_response(k);
                            
                            rigid_bodies[j].S.P = rigid_bodies[j].S.P + (dP * 1.5);
                            rigid_bodies[j].S.L = rigid_bodies[j].S.L + dL;
                        }
                    }
                    
                    moving_polygons.clear();
                }
            }
        }
        
        stop_edge_collision_test = false;
        
        // edge-edge collision
        for(int j = 0; j < edge_collision_list.size() && stop_edge_collision_test == false; j++)
        {
            process_edge_collision(j);
        }
        
        t += h;
    }
}

bool RigidBodySimulator::check_collision(void)
{
    double t_hit_tol = 0; //0.00008;
    bool collision = false;
    
    colliding_polygons.clear();
    
    for (int i = 0; i < polygon_count; i++)
    {
        Vector3 p = polygons[i].vertices[0];
        Vector3 n = polygons[i].normal;
        
        //Check collision with the plane that has the polygon
        double t_hit_new = ((p - pos)*n)/(v*n);
        
        if (t_hit_new < t_hit && t_hit_new >= t_hit_tol)
        {
            //Find the collision point
            pos_hit_new = pos + (v * t_hit_new);
            
            //Project vertices to 2D plane
            //Calculate 2D edge vectors for each projected vertex
            //Form 2X2 matrix for each vertex with edge vector and vector toward x_hit_new
            //Find the determinant of the matrix for each row
            bool same_sign = polygons[i].check_determinants(pos_hit_new);
            
            if (same_sign)
            {
                if (t_hit_new < t_hit)
                {
                    //colliding_polygons.clear();
                    t_hit = t_hit_new;
                    colliding_polygons.push_back(&polygons[i]);
                }
                collision = true;
            }
        }
    }
    return collision;
}

void RigidBodySimulator::update_collision_response(void)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        Vector3 p = colliding_polygons[i]->vertices[0];
        Vector3 n = colliding_polygons[i]->normal;
        double cr = colliding_polygons[i]->cr;
        //double cf = colliding_polygons[i]->cf;
        
        Vector3 r = pos - pos_c;
        Vector3 total_velocity_before = v + w.cross(r);
        double v_before = total_velocity_before * n;
        
        double j_numerator = -(1+cr) * v_before;
        double j_denominator = (1/m) + n * (I_inv * (r.cross(n)).cross(r));
        
        double j = j_numerator / j_denominator;
        
        Vector3 J = n*j;
        
        dP = dP + J;
        dL = dL + r.cross(J);
    }
    colliding_polygons.clear();
}

StateVectorRB RigidBodySimulator::integrate(const int& rigid_body_id, StateVectorRB SV, const double& time_step)
{
    if(euler_integration)
        return rigid_bodies[rigid_body_id].IntegrateEuler(SV, time_step);
    else
        return rigid_bodies[rigid_body_id].IntegrateRK4(SV, time_step);
}

bool RigidBodySimulator::check_vertex_face_collision(void)
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
            pos_hit_new = pos + (v * t_hit_new);
            
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

void RigidBodySimulator::update_vertex_face_collision_response(const int& mesh_id)
{
    for (int i = 0; i< colliding_polygons.size(); i++)
    {
        Vector3 pos_b = pos;    // collision point is same for a and b
        Vector3 pos_c_b = rigid_bodies[mesh_id].S.pos;
        float m_b = rigid_bodies[mesh_id].m;
        Vector3 v_b = rigid_bodies[mesh_id].S.P/m_b;
        Matrix3X3 R = rigid_bodies[mesh_id].S.q.rotation_matrix();
        Matrix3X3 I_inv_b = R * rigid_bodies[mesh_id].I0_inv * R.transpose();    // inverse of moment of inertia
        Vector3 w_b = I_inv_b * rigid_bodies[mesh_id].S.L;
        
        Vector3 p = colliding_polygons[i]->vertices[0];
        Vector3 n = colliding_polygons[i]->normal;
        double cr = colliding_polygons[i]->cr;
        //double cf = colliding_polygons[i]->cf;
        
        Vector3 r = pos - pos_c;
        Vector3 r_b = pos_b - pos_c_b;
        Vector3 total_velocity_before_a = v + w.cross(r);
        Vector3 total_velocity_before_b = v_b + w_b.cross(r_b);
        double v_rel_before = n * (total_velocity_before_a - total_velocity_before_b);
        
        if(v_rel_before < 0)
        {
            double j_numerator = -(1+cr) * v_rel_before;
            double j_denominator = (1/m) + (1/m_b) + n * (I_inv * (r.cross(n)).cross(r) + I_inv_b * (r_b.cross(n)).cross(r_b));
            
            double j = j_numerator / j_denominator;
            
            dP = dP + (n*j);
            dL = dL + (r.cross(n)*j);
            
            rigid_bodies[mesh_id].S.P = rigid_bodies[mesh_id].S.P  - (n*j);
            rigid_bodies[mesh_id].S.L = rigid_bodies[mesh_id].S.L  - (r_b.cross(n)*j);
        }
    }
    colliding_polygons.clear();
}


void RigidBodySimulator::process_edge_collision(const int& edge_collision_id)
{
    Vector3 p1, p2, q1, q2;
    Vector3 a, b, a_hat, b_hat;
    Vector3 n;
    Vector3 r;
    Vector3 m_vector;
    
    float s, t;
    
    int m1 = edge_collision_list[edge_collision_id].m1; //mesh one
    int e1 = edge_collision_list[edge_collision_id].e1; //edge one
    
    int p1_index = rigid_bodies[m1].struts[e1].v1;
    int p2_index = rigid_bodies[m1].struts[e1].v2;
    
    p1 = rigid_bodies[m1].vertices[p1_index].pos;
    p2 = rigid_bodies[m1].vertices[p2_index].pos;
    
    int m2 = edge_collision_list[edge_collision_id].m2; //mesh two
    int e2 = edge_collision_list[edge_collision_id].e2; //edge two
    
    int q1_index = rigid_bodies[m2].struts[e2].v1;
    int q2_index = rigid_bodies[m2].struts[e2].v2;
    
    q1 = rigid_bodies[m2].vertices[q1_index].pos;
    q2 = rigid_bodies[m2].vertices[q2_index].pos;
    
    if (rigid_bodies[m1].S.pos.distance(rigid_bodies[m1].S.pos) > 30)
        return;
    
    a = p2-p1;
    b = q2-q1;
    
    a_hat = a.unit_vector();
    b_hat = b.unit_vector();
    
    n = (a.cross(b)).unit_vector(); // normal
    
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
    
    m_vector = qa - pa;
    
    Vector3 m_prev = edge_collision_list[edge_collision_id].m;
    edge_collision_list[edge_collision_id].m = m_vector;
   
    if (m_vector.magnitude() > 0.02)
        return;
    else
    {
        if (m_prev * m_vector >= 0)
            return;
    }
    
    // collision detected at this point
    
    pos = pa;    // collision point is same for a and b
    pos_c = rigid_bodies[m1].S.pos;
    m = rigid_bodies[m1].m;
    v = rigid_bodies[m1].S.P/m;
    Matrix3X3 R = rigid_bodies[m1].S.q.rotation_matrix();
    I_inv = R * rigid_bodies[m1].I0_inv * R.transpose();    // inverse of moment of inertia
    w = I_inv * rigid_bodies[m1].S.L;
    
    
    Vector3 pos_b = pa;    // collision point is same for a and b
    Vector3 pos_c_b = rigid_bodies[m2].S.pos;
    float m_b = rigid_bodies[m2].m;
    Vector3 v_b = rigid_bodies[m2].S.P/m_b;
    R = rigid_bodies[m2].S.q.rotation_matrix();
    Matrix3X3 I_inv_b = R * rigid_bodies[m2].I0_inv * R.transpose();    // inverse of moment of inertia
    Vector3 w_b = I_inv_b * rigid_bodies[m2].S.L;
    
    double cr = 0.2;
    //double cf = colliding_polygons[i]->cf;
    
    Vector3 r_a = pos - pos_c;
    Vector3 r_b = pos_b - pos_c_b;
    Vector3 total_velocity_before_a = v + w.cross(r_a);
    Vector3 total_velocity_before_b = v_b + w_b.cross(r_b);
    double v_rel_before = n * (total_velocity_before_a - total_velocity_before_b);
    
    if(v_rel_before < 0)
    {
        double j_numerator = -(1+cr) * v_rel_before;
        double j_denominator = (1/m) + (1/m_b) + n * (I_inv * (r_a.cross(n)).cross(r_a) + I_inv_b * (r_b.cross(n)).cross(r_b));
        
        double j = j_numerator / j_denominator;
        
        rigid_bodies[m1].S.P = rigid_bodies[m1].S.P  + ((n*j)*2);
        rigid_bodies[m1].S.L = rigid_bodies[m1].S.L  + (r_a.cross(n)*j);
        
        rigid_bodies[m2].S.P = rigid_bodies[m2].S.P  - ((n*j)*2);
        rigid_bodies[m2].S.L = rigid_bodies[m2].S.L  - (r_b.cross(n)*j);
        
        //stop_edge_collision_test = true;
    }
}

void RigidBodySimulator::generate_cubes(const Vector3& position, const Vector3& normal)
{
    u_x = normal;
    u_z = normal.cross(Vector3(0,1,0));
    u_y = u_x.cross(u_z);
    
    R.add_vector3_as_column(u_x, 0);
    R.add_vector3_as_column(u_y, 1);
    R.add_vector3_as_column(u_z, 2);
    
    float v = ((float)rand())/((float)RAND_MAX);
    float b = 1 - v;
    
    add_cube_rigid_body(position, R.get_euler_rotation(), Vector3(-9*v*b, v*0.5*b, v*15), Vector3(v*b*0.5, v*5, b*4), (8 * (1-v)) + 3,  10*b, 0.04, 0.03, Vector3(0,1,0), "wireframe");
}

void RigidBodySimulator::add_cube_rigid_body(const Vector3& pos, const Vector3& rot,  const Vector3& velocity, const Vector3& angular_velocity, const float& radius, const float& mass, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type)
{
    rigid_bodies.push_back(RigidBody("cube", pos, rot, velocity, angular_velocity, radius, mass, drag_coefficient, lift_coefficient, mesh_color, render_type));
}

void RigidBodySimulator::prepare_moving_edge_list(void)
{
    for(int i = 0; i < rigid_bodies.size(); i++)
    {
        for(int j = 0; j < rigid_bodies[i].struts.size(); j++)
        {
            MovingEdgeInfo moving_edge_info;
            moving_edge_info.mesh_id = i;
            moving_edge_info.edge_id = j;
            moving_edge_info.is_static = false;
            moving_edges.push_back(moving_edge_info);
        }
    }
}

void RigidBodySimulator::prepare_edge_collision_list(void)
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
