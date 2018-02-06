//
//  SpringyMesh.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 10/31/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include "SpringyMesh.h"

SpringyMesh::SpringyMesh(void)
{
    
}

SpringyMesh::SpringyMesh(const std::string& type, const Vector3& initial_pos, const Vector3& initial_velocity, const float& radius, const float& mass, const float& time_constant, const float& period_of_oscillation, const float& drag_coefficient, const float& lift_coefficient, const Vector3& mesh_color, const std::string& render_type):m(mass), T(time_constant), P(period_of_oscillation), cd(drag_coefficient), cl(lift_coefficient), color(mesh_color)
{
    k = (4*3.14*3.14*m) / (P*P);
    d = (2*m)/T;
    
    if(render_type.compare("wireframe") == 0)
        render_wireframe = true;
    else if(render_type.compare("shaded") == 0)
        render_shaded_model = true;
    
    if (type.compare("cube") == 0)
    {
        // create axis-aligned cube
        
        // create 8 vertices
        Vector3* positions = new Vector3[8];
        
        positions[0] = Vector3(radius, -radius, radius) + initial_pos;
        positions[1] = Vector3(radius, -radius, -radius) + initial_pos;
        positions[2] = Vector3(radius, radius, -radius) + initial_pos;
        positions[3] = Vector3(radius, radius, radius) + initial_pos;
        positions[4] = Vector3(-radius, -radius, -radius) + initial_pos;
        positions[5] = Vector3(-radius, -radius, radius) + initial_pos;
        positions[6] = Vector3(-radius, radius, radius) + initial_pos;
        positions[7] = Vector3(-radius, radius, -radius) + initial_pos;
        
        for(int i = 0; i < 8; i++)
        {
            vertices.push_back(Vertex(positions[i], m/8, initial_velocity));
        }
        
        // initialize state vector
        for(int i = 0; i < vertices.size(); i++)
        {
            S.elements.push_back(vertices[i].pos);
        }
        
        for(int i = 0; i < vertices.size(); i++)
        {
            S.elements.push_back(vertices[i].v);
        }
        
        S.n = vertices.size();
        
        S.m = new float[S.n];
        S.f = new Vector3[S.n];
        
        for(int i = 0; i < S.n; i++)
        {
            S.m[i] = vertices[i].m;
        }
        
        K1.allocate_memory(S.n);
        K2.allocate_memory(S.n);
        K3.allocate_memory(S.n);
        K4.allocate_memory(S.n);
        
        // create 18 struts (12 along each edge and 6 along one of the two face diagonals)
        
        float l = positions[0].distance(positions[1]);  // base length based on which k and d are calculated
        
        struts.push_back(Strut(0, 1, k, d, l, positions[0].distance(positions[1])));    //0
        struts.push_back(Strut(0, 3, k, d, l, positions[0].distance(positions[3])));    //1
        struts.push_back(Strut(0, 5, k, d, l, positions[0].distance(positions[5])));    //2
        struts.push_back(Strut(0, 4, k, d, l, positions[0].distance(positions[4])));    //3
        struts.push_back(Strut(1, 3, k, d, l, positions[1].distance(positions[3])));    //4
        struts.push_back(Strut(1, 2, k, d, l, positions[1].distance(positions[2])));    //5
        struts.push_back(Strut(1, 4, k, d, l, positions[1].distance(positions[4])));    //6
        struts.push_back(Strut(2, 3, k, d, l, positions[2].distance(positions[3])));    //7
        struts.push_back(Strut(2, 4, k, d, l, positions[2].distance(positions[4])));    //8
        struts.push_back(Strut(2, 7, k, d, l, positions[2].distance(positions[7])));    //9
        struts.push_back(Strut(3, 6, k, d, l, positions[3].distance(positions[6])));    //10
        struts.push_back(Strut(3, 7, k, d, l, positions[3].distance(positions[7])));    //11
        struts.push_back(Strut(3, 5, k, d, l, positions[3].distance(positions[5])));    //12
        struts.push_back(Strut(4, 5, k, d, l, positions[4].distance(positions[5])));    //13
        struts.push_back(Strut(4, 7, k, d, l, positions[4].distance(positions[7])));    //14
        struts.push_back(Strut(4, 6, k, d, l, positions[4].distance(positions[6])));    //15
        struts.push_back(Strut(5, 6, k, d, l, positions[5].distance(positions[6])));    //16
        struts.push_back(Strut(6, 7, k, d, l, positions[6].distance(positions[7])));    //17
        
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
        
        // torsional spring face pairs
        
        //0
        add_torsional_spring(0, 3, 0);
        //faces.push_back(Face(0,4,1));       //0
        //faces.push_back(Face(3,6,0));       //3
        
        //1
        add_torsional_spring(0, 5, 1);
        //faces.push_back(Face(0,4,1));       //0
        //faces.push_back(Face(2,1,12));      //5
        
        //2
        add_torsional_spring(2, 5, 2);
        //faces.push_back(Face(2,13,3));      //2
        //faces.push_back(Face(2,1,12));      //5
        
        //3
        add_torsional_spring(2, 3, 3);
        //faces.push_back(Face(2,13,3));      //2
        //faces.push_back(Face(3,6,0));       //3
        
        //4
        add_torsional_spring(0, 1, 4);
        //faces.push_back(Face(0,4,1));       //0
        //faces.push_back(Face(5,7,4));       //1
        
        //5
        add_torsional_spring(1, 7, 5);
        //faces.push_back(Face(5,7,4));       //1
        //faces.push_back(Face(5,6,8));       //7
        
        //6
        add_torsional_spring(3, 7, 6);
        //faces.push_back(Face(3,6,0));       //3
        //faces.push_back(Face(5,6,8));       //7
        
        //7
        add_torsional_spring(1, 9, 7);
        //faces.push_back(Face(5,7,4));       //1
        //faces.push_back(Face(7,9,11));      //9
        
        //8
        add_torsional_spring(6, 7, 8);
        //faces.push_back(Face(8,14,9));      //6
        //faces.push_back(Face(5,6,8));       //7

        //9
        add_torsional_spring(6, 9, 9);
        //faces.push_back(Face(8,14,9));      //6
        //faces.push_back(Face(7,9,11));      //9
        
        //10
        add_torsional_spring(4, 8, 10);
        //faces.push_back(Face(12,10,16));    //4
        //faces.push_back(Face(11,17,10));    //8
        
        //11
        add_torsional_spring(8, 9, 11);
        //faces.push_back(Face(11,17,10));    //8
        //faces.push_back(Face(7,9,11));      //9
        
        //12
        add_torsional_spring(4, 5, 12);
        //faces.push_back(Face(12,10,16));    //4
        //faces.push_back(Face(2,1,12));      //5
        
        //13
        add_torsional_spring(2, 10, 13);
        //faces.push_back(Face(2,13,3));      //2
        //faces.push_back(Face(16,15,13));    //10
        
        //14
        add_torsional_spring(6, 11, 14);
        //faces.push_back(Face(8,14,9));      //6
        //faces.push_back(Face(15,17,14));    //11
        
        //15
        add_torsional_spring(10, 11, 15);
        //faces.push_back(Face(16,15,13));    //10
        //faces.push_back(Face(15,17,14));    //11
        
        //16
        add_torsional_spring(4, 10, 16);
        //faces.push_back(Face(12,10,16));    //4
        //faces.push_back(Face(16,15,13));    //10
        
        //17
        add_torsional_spring(8, 11, 17);
        //faces.push_back(Face(11,17,10));    //8
        //faces.push_back(Face(15,17,14));    //11
    }
}

SpringyMesh::~SpringyMesh(void)
{
    
}

void SpringyMesh::add_faces_to_polygons(void)
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

void SpringyMesh::add_torsional_spring(const int& f1, const int& f2, const int& common_strut)
{
    std::vector<int> i;
    i.push_back(struts[common_strut].v1);   //i0
    i.push_back(struts[common_strut].v2);   //i1
    
    std::vector<int> edges;
    edges.push_back(faces[f1].s1);
    edges.push_back(faces[f1].s2);
    edges.push_back(faces[f1].s3);
    
    edges.push_back(faces[f2].s1);
    edges.push_back(faces[f2].s2);
    edges.push_back(faces[f2].s3);
    
    int q = 0;
    
    do
    {
        int edge = edges[q];
        
        int v[2];
        v[0] = struts[edge].v1;
        v[1] = struts[edge].v2;
        
        for(int j = 0; j < 2; j++)
        {
            bool present = false;
            int l = 0;
            while(l < i.size() && !present)
            {
                if (v[j] == i[l])
                    present = true;
                
                ++l;
            }
            
            if(!present)
                i.push_back(v[j]);
        }
        ++q;
    }while(i.size() < 4);
    
    Vector3 x0, x1, x2, x3;
    
    x0 = S.elements[i[0]];
    x1 = S.elements[i[1]];
    x2 = S.elements[i[2]];
    x3 = S.elements[i[3]];
    
    Vector3 x01, x02, x03;
    x01 = x1-x0;
    x02 = x2-x0;
    x03 = x3-x0;
    
    Vector3 hinge = x01.unit_vector();
    
    Vector3 nl, nr;
    
    nl = (x01.cross(x02)).unit_vector();
    nr = (x03.cross(x01)).unit_vector();
    
    float sin_theta = (nl.cross(nr))*hinge;
    float cos_theta = nl*nr;
    
    float theta = atan2(sin_theta, cos_theta) * (180/3.14);
    
    torsional_springs.push_back(TorsionalSpring(i[0], i[1], i[2], i[3], theta));
}

void SpringyMesh::update_forces(const Vector3& g, const Vector3& w)
{
    // environmental forces (gravity, air-drag, friction) for each particle
    for(int i = 0; i < S.n; i++)
    {
        S.f[i] = (g * S.m[i]);// + w; // - (S.elements[i+S.n] * d);
    }
    
    // spring and damper forces for each strut
    for(int i = 0; i < struts.size(); i++)
    {
        int index1 = struts[i].v1;
        int index2 = struts[i].v2;
        
        Vector3 pos1 = S.elements[index1];
        Vector3 pos2 = S.elements[index2];
        
        Vector3 v1 = S.elements[index1 + S.n];
        Vector3 v2 = S.elements[index2 + S.n];
        
        float l_0 = struts[i].l;
        float l = pos1.distance(pos2);
    
        Vector3 vector_12 = (pos2 - pos1).unit_vector();
        
        //spring force
        Vector3 fs1 = vector_12 * (struts[i].k * (l - l_0));
        Vector3 fs2 = -fs1;
        
        S.f[index1] = S.f[index1] + fs1;
        S.f[index2] = S.f[index2] + fs2;
        
        //damping force
        Vector3 fd1 = vector_12 * ((v2-v1)*vector_12) * struts[i].d;
        Vector3 fd2 = -fd1;
        
        S.f[index1] = S.f[index1] + fd1;
        S.f[index2] = S.f[index2] + fd2;
    }
    
    // forces due to the torsional springs
    for(int i = 0; i < torsional_springs.size(); i++)
    {
        int i0, i1, i2, i3;
        Vector3 x0, x1, x2, x3;
        Vector3 v2, v3;
        float theta0;
        
        i0 = torsional_springs[i].i0;
        i1 = torsional_springs[i].i1;
        i2 = torsional_springs[i].i2;
        i3 = torsional_springs[i].i3;
        
        theta0 = torsional_springs[i].theta0;
        
        x0 = S.elements[i0];
        x1 = S.elements[i1];
        x2 = S.elements[i2];
        x3 = S.elements[i3];
        
        v2 = S.elements[i2+S.n];
        v3 = S.elements[i3+S.n];
        
        Vector3 x01, x02, x03;
        x01 = x1-x0;
        x02 = x2-x0;
        x03 = x3-x0;
        
        Vector3 hinge = x01.unit_vector();
        
        Vector3 nl, nr;
        
        nl = (x01.cross(x02)).unit_vector();
        nr = (x03.cross(x01)).unit_vector();
        
        float sin_theta = (nl.cross(nr))*hinge;
        float cos_theta = nl*nr;
        
        float theta = atan2(sin_theta, cos_theta) * (180/3.14);
        
        Vector3 tk = hinge * (k * (theta-theta0));  // torque exerted by the spring
        
        float sl, sr;   // speed of the vertices
        float rot_speed_l, rot_speed_r;
        Vector3 rl, rr; // perpendicular vectors from the hinge to the vertices
        
        rl = x02-(hinge*(x02*hinge));
        rr = x03-(hinge*(x03*hinge));
        
        float rl_length = rl.magnitude();
        float rr_length = rr.magnitude();
        
        sl = v2*nl;
        sr = v3*nr;
        
        rot_speed_l = sl/rl_length;
        rot_speed_r = sr/rr_length;
        
        Vector3 td = hinge * (-d * (rot_speed_l+rot_speed_r));  // damping torque
        
        Vector3 torque = tk + td;   // total torque
        
        Vector3 f0, f1, f2, f3; // forces acting on each vertex
        
        f2 = nl * (torque * hinge)/rl_length;
        
        f3 = nr * (torque * hinge)/rr_length;
        
        float d02, d03;
        
        d02 = x02 * hinge;
        d03 = x03 * hinge;
        
        float l01 = x0.distance(x1);
        
        f1 = -((f2*d02) + (f3*d03))/l01;
        
        f0 = -(f1+f2+f3);
        
        S.f[i0] = S.f[i0] + f0;
        S.f[i1] = S.f[i1] + f1;
        S.f[i2] = S.f[i2] + f2;
        S.f[i3] = S.f[i3] + f3;
    }
    
    // lift and drag forces acting on the faces
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
        Vector3 v1 = S.elements[vert1 + S.n];
        Vector3 v2 = S.elements[vert2 + S.n];
        Vector3 v3 = S.elements[vert3 + S.n];
        
        Vector3 v = (v1 + v2 + v3)/3;   // approximate velocity at the centre for nearly equilateral triangles
        
        Vector3 v_r = v - w;    // relative velocity
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
        S.f[common_vertex] = S.f[common_vertex] + (f * (theta1/180));
        
        //angle between strut two and strut three
        float theta2 = find_angle_and_common_vertex(edge_index2, edge_index3, common_vertex);
        S.f[common_vertex] = S.f[common_vertex] + (f * (theta2/180));
        
        //angle between strut one and strut three
        float theta3 = find_angle_and_common_vertex(edge_index1, edge_index3, common_vertex);
        S.f[common_vertex] = S.f[common_vertex] + (f * (theta3/180));
    }
}

Vector3 SpringyMesh::find_normal_and_vertex_indices(const int& e1, const int& e2, int& v1, int& v2, int& v3)
{
    Vector3 pos1, pos2, pos3; // positions
    Vector3 edge1, edge2;   // edge vectors
    
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
    
    pos1 = S.elements[v1];
    pos2 = S.elements[v2];
    pos3 = S.elements[v3];
    
    edge1 = pos2 - pos1;
    edge2 = pos3 - pos1;
    
    return edge2.cross(edge1);
    //return edge1.cross(edge2);
}

float SpringyMesh::find_angle_and_common_vertex(const int& e1, const int& e2, int& common_vertex)
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
    
    pos1 = S.elements[v1];
    pos2 = S.elements[v2];
    pos3 = S.elements[v3];
    
    edge1 = pos2 - pos1;
    edge2 = pos3 - pos1;
    
    //find the angle
    return edge1.angle(edge2) * (180/3.14);
}
