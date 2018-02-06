//
//  RigidBody.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 11/12/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "RigidBody.h"

RigidBody::RigidBody(void)
{
    
}

RigidBody::RigidBody(const std::string& type, const Vector3& initial_pos, const Vector3& initial_rot, const Vector3& initial_velocity, const Vector3& initial_angular_velocity, const float& radius, const float& mass, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type):m(mass), color(mesh_color), cd(drag_coefficient), cl(lift_coefficient)
{
    if(render_type.compare("wireframe") == 0)
        render_wireframe = true;
    else if(render_type.compare("shaded") == 0)
        render_shaded_model = true;
    
    if (type.compare("cube") == 0)
    {
        // create axis-aligned cube
        
        // create 8 vertices
        Vector3* positions = new Vector3[8];
        
        r = radius;
        
        positions[0] = Vector3(radius, -radius, radius);
        positions[1] = Vector3(radius, -radius, -radius);
        positions[2] = Vector3(radius, radius, -radius);
        positions[3] = Vector3(radius, radius, radius);
        positions[4] = Vector3(-radius, -radius, -radius);
        positions[5] = Vector3(-radius, -radius, radius);
        positions[6] = Vector3(-radius, radius, radius);
        positions[7] = Vector3(-radius, radius, -radius);
        
        for(int i = 0; i < 8; i++)
        {
            vertices.push_back(Vertex(positions[i], m/8));
        }
        
        // create 18 struts (12 along each edge and 6 along one of the two face diagonals)
        
        struts.push_back(Strut(0, 1));    //0
        struts.push_back(Strut(0, 3));    //1
        struts.push_back(Strut(0, 5));    //2
        struts.push_back(Strut(0, 4));    //3
        struts.push_back(Strut(1, 3));    //4
        struts.push_back(Strut(1, 2));    //5
        struts.push_back(Strut(1, 4));    //6
        struts.push_back(Strut(2, 3));    //7
        struts.push_back(Strut(2, 4));    //8
        struts.push_back(Strut(2, 7));    //9
        struts.push_back(Strut(3, 6));    //10
        struts.push_back(Strut(3, 7));    //11
        struts.push_back(Strut(3, 5));    //12
        struts.push_back(Strut(4, 5));    //13
        struts.push_back(Strut(4, 7));    //14
        struts.push_back(Strut(4, 6));    //15
        struts.push_back(Strut(5, 6));    //16
        struts.push_back(Strut(6, 7));    //17
        
        // create 12 faces (2 triangles for each of the six sides)
        // the struts are ordered in anti-clockwise direction when viewed from outside of the cube
        
        faces.push_back(Face(0,4,1));       //0
        faces.push_back(Face(5,7,4));       //1
        faces.push_back(Face(2,13,3));      //2
        faces.push_back(Face(3,6,0));       //3
        faces.push_back(Face(12,10,16));    //4
        faces.push_back(Face(2,1,12));      //5
        faces.push_back(Face(8,14,9));      //6
        faces.push_back(Face(5,6,8));       //7
        faces.push_back(Face(11,17,10));    //8
        faces.push_back(Face(7,9,11));      //9
        faces.push_back(Face(16,15,13));    //10
        faces.push_back(Face(15,17,14));    //11
        
        add_faces_to_polygons();
                         
        // initialize state vector
        S.pos = initial_pos;
        S.q.from_euler(initial_rot.x, initial_rot.y, initial_rot.z);
        
        // center of mass
        /*     // May be useful code for other type of meshes
        Vector3 numerator;
        double denominator;
       
        for(int i = 0; i < 8; i++)
        {
            numerator = numerator + vertices[i].pos * vertices[i].m;
            denominator = denominator + vertices[i].m;
        }
        
        S.c = numerator/denominator;
        
        std::cout << "Center of mass: ";
        S.c.print();
         */
         
        // initial inertia
        float a = radius*2;
        float element = m/12 * (a*a + a*a);   // l = w = h = a
        Matrix3X3 I0;
        I0.elements = {{element,0,0},
                       {0,element,0},
                       {0,0,element}};
        I0_inv = I0.inverse_matrix();
        
        // linear momentum
        S.P = initial_velocity * m;
        
        // angular momentum
        S.L = I0 * initial_angular_velocity;
        
        compute_transformation_matrix();
        update_vertices();
    }
}

RigidBody::~RigidBody(void)
{
    
}

void RigidBody::add_faces_to_polygons(void)
{
    for(int l = 0; l < faces.size(); l++)
    {
        int e1 = faces[l].s1;
        int e2 = faces[l].s2;
        int e3 = faces[l].s3;
        
        int vert1, vert2, vert3;
        
        int e1v1 = struts[e1].v1;
        int e1v2 = struts[e1].v2;
        
        int e2v1 = struts[e2].v1;
        int e2v2 = struts[e2].v2;
        
        int e3v1 = struts[e3].v1;
        int e3v2 = struts[e3].v2;
        
        if(e1v1 == e2v1 || e1v1 == e2v2)
            vert1 = e1v1;
        else if(e1v2 == e2v1 || e1v2 == e2v2)
            vert1 = e1v2;
        
        if(e2v1 == e3v1 || e2v1 == e3v2)
            vert2 = e2v1;
        else if(e2v2 == e3v1 || e2v2 == e3v2)
            vert2 = e2v2;
        
        if(e3v1 == e1v1 || e3v1 == e1v2)
            vert3 = e3v1;
        else if(e3v2 == e1v1 || e3v2 == e1v2)
            vert3 = e3v2;
        
        Polygon polygon;
        polygon.v1 = vert1;
        polygon.v2 = vert2;
        polygon.v3 = vert3;
        polygons.push_back(polygon);
    }
}

void RigidBody::update_forces(const Vector3& g, const Vector3& wind)
{
    forces.clear();
    
    // environmental forces (gravity, air-drag, friction) for each particle
    force_node = new force;
    force_node->f = g * m;
    force_node->is_body_force = true;
    force_node->point = Vector3(0,0,0);
    forces.push_back(*force_node);
    
    // lift and drag forces acting on the faces
    //This code works. Except that the force needs to be applied at the center of the face rather than the vertices
    /*
    for(int i = 0; i < faces.size(); i++)
    {
        int edge_index1 = faces[i].s1;
        int edge_index2 = faces[i].s2;
        int edge_index3 = faces[i].s3;
        
        int vert1, vert2, vert3;
        
        Vector3 n = find_normal_and_vertex_indices(edge_index1, edge_index2, vert1, vert2, vert3);  // normal
        float A = 0.5 * n.magnitude();  // area of the triangle (half the magnitude of cross product between any two edges)
        n.normalize();
        
        //velocity
        
        Vector3 v = S.P/m;
        
        Vector3 v_r = v - wind;    // relative velocity
        v_r.normalize();
        
        float effective_area = A * (n * v_r);
        
        //drag force
        Vector3 fd = v_r * (-cd * effective_area);
        
        //lift force
        Vector3 n_cross_vr = (n.cross(v_r)).unit_vector();
        Vector3 fl = (v_r.cross(n_cross_vr)) * (-cl * effective_area);
       
        Vector3 f = fl + fd;
        
        int common_vertex;
        
        //angle between strut one and strut two
        float theta1 = find_angle_and_common_vertex(edge_index1, edge_index2, common_vertex);
        //S.f[common_vertex] = S.f[common_vertex] + (f * (theta1/180));
        
        force_node = new force;
        force_node->f = (f * (theta1/180));
        force_node->is_body_force = false;
        force_node->point = vertices[common_vertex].pos;
        forces.push_back(*force_node);
        
        //angle between strut two and strut three
        float theta2 = find_angle_and_common_vertex(edge_index2, edge_index3, common_vertex);
        //S.f[common_vertex] = S.f[common_vertex] + (f * (theta2/180));
        
        force_node = new force;
        force_node->f = (f * (theta2/180));
        force_node->is_body_force = false;
        force_node->point = vertices[common_vertex].pos;
        forces.push_back(*force_node);
        
        //angle between strut one and strut three
        float theta3 = find_angle_and_common_vertex(edge_index1, edge_index3, common_vertex);
        //S.f[common_vertex] = S.f[common_vertex] + (f * (theta3/180));
        
        force_node = new force;
        force_node->f = (f * (theta3/180));
        force_node->is_body_force = false;
        force_node->point = vertices[common_vertex].pos;
        forces.push_back(*force_node);
    }
    */
}

StateVectorRB RigidBody::IntegrateEuler(StateVectorRB SV, const double& h)
{
    return SV + F(SV) * h;
}

StateVectorRB RigidBody::IntegrateRK4(StateVectorRB SV, const double& h)
{
    K1 = F(SV);
    K2 = F(SV + K1*(h*0.5));
    K3 = F(SV + K2*(h*0.5));
    K4 = F(SV + K3*h);
    
    return SV + (K1 + (K2*2) + (K3*2) + K4)*(double)(h/6);
}

StateVectorRB RigidBody::F(StateVectorRB SV)
{
    StateVectorRB S_dr;
    
    S_dr.pos = SV.P/m;
    
    Matrix3X3 R = SV.q.rotation_matrix();
    Matrix3X3 I_inv = R * I0_inv * R.transpose();    // inverse of moment of inertia
    Vector3 w =  I_inv * SV.L;
    
    S_dr.q = SV.q * Quaternion(0, w) * 0.5;     // w => angular velocity
    
    S_dr.P = Vector3(0,0,0);
    S_dr.L = Vector3(0,0,0);
    
    for(int i = 0; i < forces.size(); i++)
    {
        S_dr.P = S_dr.P + forces[i].f;
        
        if(!forces[i].is_body_force)
        {
            Vector3 r = forces[i].point - SV.pos;
            S_dr.L = S_dr.L + r.cross(forces[i].f);
        }
    }
    return S_dr;
}

void RigidBody::compute_transformation_matrix(void)
{
    T.set_translation(S.pos);
    S.q.normalize();
    R.from_quaternion(S.q);
    TR = T * R;
}

void RigidBody::update_vertices(void)
{
    for(int i = 0; i < vertices.size(); i++)
        vertices[i].pos = TR.multiply_with_point3d(vertices[i].pos0);
}

Vector3 RigidBody::find_normal_and_vertex_indices(const int& e1, const int& e2, int& v1, int& v2, int& v3)
{
    Vector3 pos1, pos2, pos3;   // positions
    Vector3 edge1, edge2;       // edge vectors
    
    int e1_v1 = struts[e1].v1;
    int e1_v2 = struts[e1].v2;
    
    int e2_v1 = struts[e2].v1;
    int e2_v2 = struts[e2].v2;
    
    if(e1_v1 == e2_v1 || e1_v1 == e2_v2)
    {
        v1 = e1_v1;
        v2 = e1_v2;
    }
    else
    {
        v1 = e1_v2;
        v2 = e1_v1;
    }
    
    if(v1 == e2_v1)
        v3 = e2_v2;
    else
        v3 = e2_v1;
    
    pos1 = vertices[v1].pos;
    pos2 = vertices[v2].pos;
    pos3 = vertices[v3].pos;
    
    edge1 = pos2 - pos1;
    edge2 = pos3 - pos1;
    
    return edge2.cross(edge1);
}

float RigidBody::find_angle_and_common_vertex(const int& e1, const int& e2, int& common_vertex)
{
    int v1, v2, v3;             // vertex indices
    Vector3 pos1, pos2, pos3;   // positions
    Vector3 edge1, edge2;       // edge vectors
    
    int e1_v1 = struts[e1].v1;
    int e1_v2 = struts[e1].v2;
    
    int e2_v1 = struts[e2].v1;
    int e2_v2 = struts[e2].v2;
    
    if(e1_v1 == e2_v1 || e1_v1 == e2_v2)
    {
        common_vertex = e1_v1;
        v1 = e1_v1;
        v2 = e1_v2;
    }
    else
    {
        common_vertex = e1_v2;
        v1 = e1_v2;
        v2 = e1_v1;
    }
    
    if(v1 == e2_v1)
        v3 = e2_v2;
    else
        v3 = e2_v1;
    
    pos1 = vertices[v1].pos;
    pos2 = vertices[v2].pos;
    pos3 = vertices[v3].pos;
    
    edge1 = pos2 - pos1;
    edge2 = pos3 - pos1;
    
    //find the angle
    return edge1.angle(edge2) * (180/3.14);
}
