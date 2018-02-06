//
//  main.cpp
//  PhysicallyBasedModeling
//
//  Created by Shyam Prathish Sargunam on 9/13/17.
//  Copyright © 2017 Shyam Prathish Sargunam. All rights reserved.
//

#include <iostream>
#include <time.h>
#include <vector>
#include <fstream>
#include "Quaternion.h"
#include "BallSimulator.h"
#include "ParticleSimulator.h"
#include "SPHSimulator.h"
#include "FlockSimulator.h"
#include "SpringyMeshSimulator.h"
#include "RigidBodySimulator.h"
#include "Polygon.h"

#include "json.hpp"
using json = nlohmann::json;


#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace std;

#define SPACEBAR 32
#define UP_ARROW 72
#define DOWN_ARROW 80

int rotateon;

double xmin, xmax, ymin, ymax, zmin, zmax;
double maxdiff;

int lastx, lasty;
int xchange, ychange;
float spin = 13;
float spinup = -31.0;

int fps = 30;
double frameDuration;
clock_t startTime;
bool simulate = false;
int i = 0;

std::vector <BallSimulator> ball_simulators;
std::vector <ParticleSimulator> particle_simulators;
std::vector <SPHSimulator> SPH_simulators;
std::vector <FlockSimulator> flock_simulators;
std::vector <SpringyMeshSimulator> springy_mesh_simulators;
std::vector <RigidBodySimulator> rigid_body_simulators;

Polygon *polygons;
int polygon_count;

bool render_container = false;

Polygon *paper_plane;

Vortex* vortex;

bool trails= false;

Vector3 colr00(0.1, 0.0, 0.0);
Vector3 col0g0(0.0, 0.1, 0.0);
Vector3 col00b(0.0, 0.0, 0.1);
Vector3 colrg0(0.1, 0.1, 0.0);
Vector3 colr0b(0.1, 0.0, 0.1);
Vector3 col0gb(0.0, 0.1, 0.1);
Vector3 col_orange(0.1, 0.05, 0.0);
Vector3 col00bbb(0.0, 0.0, 0.2);

const float TO_RADIANS_MULTIPLIER = 3.14159265359 / 180;

struct Frame
{
    Vector3 pos;
    Vector3 normal;
    bool generate = false;
};

Frame* frames;
int frame_count;
int frame_id;
bool play_from_file = false;
bool generate_particles = true;

bool write_to_file = false;
int write_frame_id = 0;
int write_frame_count = 500;
std::string write_content;
std::string write_file_name = "Outputs/SPH.txt";

// Open file
std::ofstream out_file_Stream(write_file_name);


void StartTimer(void)
{
    std::cout<<"StartTimer\n";
    startTime = clock();
    frameDuration = (double)1/fps;
    simulate = true;
}

void create_cube(void)
{
    polygon_count = 6;
    polygons = new Polygon[polygon_count];
    
    //top face
    polygons[0].add_vertex(100, 100, 100);
    polygons[0].add_vertex(-100, 100, 100);
    polygons[0].add_vertex(-100, 100, -100);
    polygons[0].add_vertex(100, 100, -100);
    polygons[0].pre_compute_values();
    polygons[0].color = colr00;
    
    //bottom face
    polygons[1].add_vertex(100, -100, -100);
    polygons[1].add_vertex(-100, -100, -100);
    polygons[1].add_vertex(-100, -100, 100);
    polygons[1].add_vertex(100, -100, 100);
    polygons[1].pre_compute_values();
    polygons[1].color = col0g0;
    
    //left face
    polygons[2].add_vertex(-100, -100, 100);
    polygons[2].add_vertex(-100, -100, -100);
    polygons[2].add_vertex(-100, 100, -100);
    polygons[2].add_vertex(-100, 100, 100);
    polygons[2].pre_compute_values();
    polygons[2].color = col00b;
    
    //right face
    polygons[3].add_vertex(100, -100, -100);
    polygons[3].add_vertex(100, -100, 100);
    polygons[3].add_vertex(100, 100, 100);
    polygons[3].add_vertex(100, 100, -100);
    polygons[3].pre_compute_values();
    polygons[3].color = colrg0;
    
    //front face
    polygons[4].add_vertex(100, -100, 100);
    polygons[4].add_vertex(-100, -100, 100);
    polygons[4].add_vertex(-100, 100, 100);
    polygons[4].add_vertex(100, 100, 100);
    polygons[4].pre_compute_values();
    polygons[4].color = col0gb;
    
    //back face
    polygons[5].add_vertex(100, -100, -100);
    polygons[5].add_vertex(100, 100, -100);
    polygons[5].add_vertex(-100, 100, -100);
    polygons[5].add_vertex(-100, -100, -100);
    polygons[5].pre_compute_values();
    polygons[5].color = colr0b;
}

void create_hexagon_3d(void)
{
    polygon_count = 8;
    polygons = new Polygon[polygon_count];
    
    //top face
    polygons[0].add_vertex(100, 100, 100);
    polygons[0].add_vertex(-100, 100, 100);
    polygons[0].add_vertex(-100, 100, -100);
    polygons[0].add_vertex(100, 100, -100);
    polygons[0].pre_compute_values();
    polygons[0].color = colr00;
    
    //bottom face
    polygons[1].add_vertex(100, -100, -100);
    polygons[1].add_vertex(-100, -100, -100);
    polygons[1].add_vertex(-100, -100, 100);
    polygons[1].add_vertex(100, -100, 100);
    polygons[1].pre_compute_values();
    polygons[1].color = col0g0;
    
    //front face
    polygons[2].add_vertex(100, -100, 100);
    polygons[2].add_vertex(-100, -100, 100);
    polygons[2].add_vertex(-150, 0, 100);
    polygons[2].add_vertex(-100, 100, 100);
    polygons[2].add_vertex(100, 100, 100);
    polygons[2].add_vertex(150, 0, 100);
    polygons[2].pre_compute_values();
    polygons[2].color = col00b;
    
    //back face
    polygons[3].add_vertex(100, -100, -100);
    polygons[3].add_vertex(150, 0, -100);
    polygons[3].add_vertex(100, 100, -100);
    polygons[3].add_vertex(-100, 100, -100);
    polygons[3].add_vertex(-150, 0, -100);
    polygons[3].add_vertex(-100, -100, -100);
    polygons[3].pre_compute_values();
    polygons[3].color = colrg0;
    
    //left-top face
    polygons[4].add_vertex(-150, 0, 100);
    polygons[4].add_vertex(-150, 0, -100);
    polygons[4].add_vertex(-100, 100, -100);
    polygons[4].add_vertex(-100, 100, 100);
    polygons[4].pre_compute_values();
    polygons[4].color = col0gb;
    
    //left-bottom face
    polygons[5].add_vertex(-100, -100, 100);
    polygons[5].add_vertex(-100, -100, -100);
    polygons[5].add_vertex(-150, 0, -100);
    polygons[5].add_vertex(-150, 0, 100);
    polygons[5].pre_compute_values();
    polygons[5].color = colr0b;
    
    //right-top face
    polygons[6].add_vertex(150, 0, -100);
    polygons[6].add_vertex(150, 0, 100);
    polygons[6].add_vertex(100, 100, 100);
    polygons[6].add_vertex(100, 100, -100);
    polygons[6].pre_compute_values();
    polygons[7].color = col0g0;
    
    //right-bottom face
    polygons[7].add_vertex(100, -100, -100);
    polygons[7].add_vertex(100, -100, 100);
    polygons[7].add_vertex(150, 0, 100);
    polygons[7].add_vertex(150, 0, -100);
    polygons[7].pre_compute_values();
    polygons[7].color = col00bbb;
}

void create_hexagon_3d_with_rectangles(void)
{
    polygon_count = 9;
    polygons = new Polygon[polygon_count];
    
    //top face
    polygons[0].add_vertex(100, 100, 100);
    polygons[0].add_vertex(-100, 100, 100);
    polygons[0].add_vertex(-100, 100, -100);
    polygons[0].add_vertex(100, 100, -100);
    polygons[0].pre_compute_values();
    polygons[0].color = colr00;
    
    //bottom face
    polygons[1].add_vertex(100, -100, -100);
    polygons[1].add_vertex(-100, -100, -100);
    polygons[1].add_vertex(-100, -100, 100);
    polygons[1].add_vertex(100, -100, 100);
    polygons[1].pre_compute_values();
    polygons[1].color = col0g0;
    
    //front face
    polygons[2].add_vertex(100, -100, 100);
    polygons[2].add_vertex(-100, -100, 100);
    polygons[2].add_vertex(-150, 0, 100);
    polygons[2].add_vertex(-100, 100, 100);
    polygons[2].add_vertex(100, 100, 100);
    polygons[2].add_vertex(150, 0, 100);
    polygons[2].pre_compute_values();
    polygons[2].color = col00b;
    
    //back face
    polygons[3].add_vertex(100, -100, -100);
    polygons[3].add_vertex(150, 0, -100);
    polygons[3].add_vertex(100, 100, -100);
    polygons[3].add_vertex(-100, 100, -100);
    polygons[3].add_vertex(-150, 0, -100);
    polygons[3].add_vertex(-100, -100, -100);
    polygons[3].pre_compute_values();
    polygons[3].color = colrg0;
    
    //left-top face
    polygons[4].add_vertex(-150, 0, 100);
    polygons[4].add_vertex(-150, 0, -100);
    polygons[4].add_vertex(-100, 100, -100);
    polygons[4].add_vertex(-100, 100, 100);
    polygons[4].pre_compute_values();
    polygons[4].color = col0gb;
    
    //left-bottom face
    polygons[5].add_vertex(-100, -100, 100);
    polygons[5].add_vertex(-100, -100, -100);
    polygons[5].add_vertex(-150, 0, -100);
    polygons[5].add_vertex(-150, 0, 100);
    polygons[5].pre_compute_values();
    polygons[5].color = colr0b;
    
    //right-top face
    polygons[6].add_vertex(150, 0, -100);
    polygons[6].add_vertex(150, 0, 100);
    polygons[6].add_vertex(100, 100, 100);
    polygons[6].add_vertex(100, 100, -100);
    polygons[6].pre_compute_values();
    polygons[7].color = col0g0;
    
    //right-bottom face
    polygons[7].add_vertex(100, -100, -100);
    polygons[7].add_vertex(100, -100, 100);
    polygons[7].add_vertex(150, 0, 100);
    polygons[7].add_vertex(150, 0, -100);
    polygons[7].pre_compute_values();
    polygons[7].color = col00bbb;
    
    //rectangle
    polygons[8].add_vertex(0, -70, -50);
    polygons[8].add_vertex(0, -70, 50);
    polygons[8].add_vertex(75, -70, 50);
    polygons[8].add_vertex(75, -70, -50);
    polygons[8].pre_compute_values();
    polygons[8].color = colr00;
}

void create_triangles(void)
{
    polygon_count = 1;
    polygons = new Polygon[polygon_count];
    
    //top face
    polygons[0].add_vertex(50, 0, 50);
    polygons[0].add_vertex(0, 50, 0);
    polygons[0].add_vertex(-50, 0, 50);
    polygons[0].pre_compute_values();
    polygons[0].color = Vector3(1,1,1);
}

void create_paper_plane(void)
{
    paper_plane = new Polygon[2];
    
    //body and wings
    paper_plane[0].add_vertex(5, 0, 0);
    paper_plane[0].add_vertex(-5, 0, 2.5);
    paper_plane[0].add_vertex(-5, 0, -2.5);
    
    //tail
    paper_plane[1].add_vertex(-5, 6, 0);
    paper_plane[1].add_vertex(-2.5, 0, 0);
    paper_plane[1].add_vertex(-5, 0, 0);
}

void export_file()
{
    out_file_Stream << write_content;
    out_file_Stream.close();
    std::cout <<"exported\n";
}

void add_to_output_file(const int& particle_index, const Vector3& position, const Vector3& color)
{
    write_content += std::to_string(write_frame_id) + " "
                    + std::to_string(particle_index) + " "
                    + std::to_string(position.x) + " "
                    + std::to_string(position.y) + " "
                    + std::to_string(position.z) + " "
                    + std::to_string(color.x) + " "
                    + std::to_string(color.y) + " "
                    + std::to_string(color.z) + "\n";
}

void add_to_output_file_flocking(const int& particle_index, const Vector3& position, const Vector3& rotation, const Vector3& color)
{
    write_content += std::to_string(write_frame_id) + " "
    + std::to_string(particle_index) + " "
    + std::to_string(position.x) + " "
    + std::to_string(position.y) + " "
    + std::to_string(position.z) + " "
    + std::to_string(rotation.x) + " "
    + std::to_string(rotation.y) + " "
    + std::to_string(rotation.z) + " "
    + std::to_string(color.x) + " "
    + std::to_string(color.y) + " "
    + std::to_string(color.z) + "\n";
}

void add_to_output_rigid_body_simulation(const int& rigid_body_index, const Vector3& position, const Quaternion& rotation, const float& radius)
{
    write_content += std::to_string(write_frame_id) + " "
    + std::to_string(rigid_body_index) + " "
    + std::to_string(position.x) + " "
    + std::to_string(position.y) + " "
    + std::to_string(position.z) + " "
    + std::to_string(rotation.s) + " "
    + std::to_string(rotation.u.x) + " "
    + std::to_string(rotation.u.y) + " "
    + std::to_string(rotation.u.z) + " "
    + std::to_string(radius) + "\n";
}

void display(void)
{
    GLfloat box_diffuse[] = { 0.5, 0.5, 0.5 };
    GLfloat box_specular[] = { 0.1, 0.1, 0.1 };
    GLfloat box_shininess[] = { 1.0 };
    
    //GLfloat ball_ambient[] = { 0.4, 0.0, 0.0 };
    //GLfloat ball_diffuse[] = { 0.3, 0.0, 0.0 };
    GLfloat ball_specular[] = { 0.3, 0.3, 0.3 };
    GLfloat ball_shininess[] = { 10.0 };
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glPushMatrix();
    
    //rotate the view
    glRotatef(spinup, 1.0, 0.0, 0.0);
    glRotatef(spin, 0.0, 1.0, 0.0);
    
    
    //draw the container
    if(render_container)
    {
        for(int i = 0; i < polygon_count; i++)
        {
            glBegin(GL_POLYGON);
            GLfloat polygon_color[] = {(float)polygons[i].color.x, (float)polygons[i].color.y, (float)polygons[i].color.z};
            
            glMaterialfv(GL_FRONT, GL_AMBIENT, polygon_color);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, box_diffuse);
            glMaterialfv(GL_FRONT, GL_SPECULAR, box_specular);
            glMaterialfv(GL_FRONT, GL_SHININESS, box_shininess);
            
            Vector3* vertex = polygons[i].fetch_next_vertex();
            while(vertex != nullptr)
            {
                glVertex3f((float)vertex->x, (float)vertex->y, (float)vertex->z);
                vertex = polygons[i].fetch_next_vertex();
            }
            glEnd();
        }
    }
    
    //Particles
    
    if(particle_simulators.size() > 0)
    {
        //draw the generators
        for(int j = 0; j < particle_simulators[0].generators.size(); j++)
        {
            Polygon* generator_polygon = (particle_simulators[0].generators[j]->generator_polygon);
            
            glBegin(GL_POLYGON);
            GLfloat polygon_color[] = {(float)generator_polygon->color.x, (float)generator_polygon->color.y, (float)generator_polygon->color.z};
            
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, polygon_color);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, box_diffuse);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, box_specular);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, box_shininess);
            
            Vector3* vertex = generator_polygon->fetch_next_vertex();
            while(vertex != nullptr)
            {
                glVertex3f((float)vertex->x, (float)vertex->y, (float)vertex->z);
                vertex = generator_polygon->fetch_next_vertex();
            }
            glEnd();
        }
        
        //draw the particles
        for (int j = 0; j < particle_simulators[0].particles.size(); j++)
        {
            if(particle_simulators[0].particles[j].is_active)
            {
                if(write_to_file && write_frame_id < write_frame_count)
                {
                    add_to_output_file(j, particle_simulators[0].particles[j].pos, particle_simulators[0].particles[j].color);
                }
                
                Vector3 start_point = particle_simulators[0].particles[j].streak_start_pos;
                Vector3 end_point = particle_simulators[0].particles[j].pos;
                Vector3 streak_color = particle_simulators[0].particles[j].color;
                
                GLfloat line_ambient[] = {(float)streak_color.x, (float)streak_color.y, (float)streak_color.z};
                
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, line_ambient);
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, line_ambient);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
                
                glBegin(GL_LINES);
                glVertex3f(start_point.x, start_point.y, start_point.z);
                glVertex3f(end_point.x, end_point.y, end_point.z);
                glEnd();
            }
        }
    }
    
    //SPH Particles
    
    if(SPH_simulators.size() > 0)
    {
        //draw the generators
        for(int j = 0; j < SPH_simulators[0].generators.size(); j++)
        {
            if(write_to_file && write_frame_id < write_frame_count)
            {
                add_to_output_file(j, SPH_simulators[0].particles[j].pos, SPH_simulators[0].particles[j].color);
            }
            
            Polygon* generator_polygon = (SPH_simulators[0].generators[j]->generator_polygon);
            
            glBegin(GL_POLYGON);
            GLfloat polygon_color[] = {(float)generator_polygon->color.x, (float)generator_polygon->color.y, (float)generator_polygon->color.z};
            
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, polygon_color);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, box_diffuse);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, box_specular);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, box_shininess);
            
            Vector3* vertex = generator_polygon->fetch_next_vertex();
            while(vertex != nullptr)
            {
                glVertex3f((float)vertex->x, (float)vertex->y, (float)vertex->z);
                vertex = generator_polygon->fetch_next_vertex();
            }
            glEnd();
        }
        
        //draw the SPH particles
        for (int j = 0; j < SPH_simulators[0].particles.size(); j++)
        {
            if(SPH_simulators[0].particles[j].is_active)
            {
                if(write_to_file && write_frame_id < write_frame_count)
                {   
                    add_to_output_file(j, SPH_simulators[0].particles[j].pos, SPH_simulators[0].particles[j].color);
                }
                
                Vector3 SPH_particle_pos = SPH_simulators[0].particles[j].pos;
                Vector3 streak_color = SPH_simulators[0].particles[j].color;
                float radius = 3;
                
                GLfloat line_ambient[] = {(float)streak_color.x, (float)streak_color.y, (float)streak_color.z};
                
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, line_ambient);
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, line_ambient);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
                
                glPushMatrix();
                glTranslatef(SPH_particle_pos.x, SPH_particle_pos.y, SPH_particle_pos.z);
                glutSolidSphere(radius, radius*2, radius*2);
                glPopMatrix();
            }
        }
    }
    
    //Boids
    if(flock_simulators.size() > 0)
    {
        //draw the generators
        /*
        for(int j = 0; j < flock_simulators[0].generators.size(); j++)
        {
            Polygon* generator_polygon = (flock_simulators[0].generators[j]->generator_polygon);
            
            glBegin(GL_POLYGON);
            GLfloat polygon_color[] = {(float)generator_polygon->color.x, (float)generator_polygon->color.y, (float)generator_polygon->color.z};
            
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, polygon_color);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, box_diffuse);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, box_specular);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, box_shininess);
            
            Vector3* vertex = generator_polygon->fetch_next_vertex();
            while(vertex != nullptr)
            {
                glVertex3f((float)vertex->x, (float)vertex->y, (float)vertex->z);
                vertex = generator_polygon->fetch_next_vertex();
            }
            glEnd();
        }
         */
        
        //draw the boids
        for (int j = 0; j < flock_simulators[0].particles.size(); j++)
        {
            if(flock_simulators[0].particles[j].is_active)
            {
                if(flock_simulators[0].particles[j].is_streak)
                {
                    if(write_to_file && write_frame_id < write_frame_count && j > 0)
                    {
                        add_to_output_file_flocking(j-1, flock_simulators[0].particles[j].pos, flock_simulators[0].particles[j].l_w.get_euler_rotation(), flock_simulators[0].particles[j].start_color);
                    }
                    
                    Vector3 start_point = flock_simulators[0].particles[j].streak_start_pos;
                    Vector3 end_point = flock_simulators[0].particles[j].pos;
                    
                    Vector3 streak_color = flock_simulators[0].particles[j].color;
                    
                    GLfloat line_ambient[] = {(float)streak_color.x, (float)streak_color.y, (float)streak_color.z};
                    
                    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, line_ambient);
                    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, line_ambient);
                    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
                    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
                    
                    glBegin(GL_LINES);
                    glVertex3f(start_point.x, start_point.y, start_point.z);
                    glVertex3f(end_point.x, end_point.y, end_point.z);
                    glEnd();
                }
                else if (flock_simulators[0].particles[j].is_paper_plane)
                {
                    //draw body and wings;
                    
                    if(write_to_file && write_frame_id < write_frame_count && j > 0)
                    {
                        add_to_output_file_flocking(j-1, flock_simulators[0].particles[j].pos, flock_simulators[0].particles[j].l_w.get_euler_rotation(), flock_simulators[0].particles[j].start_color);
                    }
                    
                    
                    glBegin(GL_POLYGON);
                    Vector3 plane_body_color = flock_simulators[0].particles[j].start_color;
                    GLfloat polygon_color1[] = {(float)plane_body_color.x, (float)plane_body_color.y, (float)plane_body_color.z};
                    
                    glMaterialfv(GL_FRONT, GL_AMBIENT, polygon_color1);
                    glMaterialfv(GL_FRONT, GL_DIFFUSE, box_diffuse);
                    glMaterialfv(GL_FRONT, GL_SPECULAR, box_specular);
                    glMaterialfv(GL_FRONT, GL_SHININESS, box_shininess);
                    
                    glMaterialfv(GL_BACK, GL_AMBIENT, polygon_color1);
                    glMaterialfv(GL_BACK, GL_DIFFUSE, box_diffuse);
                    glMaterialfv(GL_BACK, GL_SPECULAR, box_specular);
                    glMaterialfv(GL_BACK, GL_SHININESS, box_shininess);
                    
                    Vector3 vertices[3];
                    
                    for(int k = 0; k < 3; k++)
                    {
                        vertices[k] = flock_simulators[0].particles[j].transform_point(paper_plane[0].vertices[k]);
                        glVertex3f((float)vertices[k].x, (float)vertices[k].y, (float)vertices[k].z);
                    }
                    glEnd();
                    
                    glBegin(GL_POLYGON);
                    for(int k = 2; k >= 0; k--)
                    {
                        glVertex3f((float)vertices[k].x, (float)vertices[k].y, (float)vertices[k].z);
                    }
                    glEnd();
                    
                    //draw tail;
                    glBegin(GL_POLYGON);
                    Vector3 plane_tail_color = flock_simulators[0].particles[j].end_color;
                    GLfloat polygon_color2[] = {(float)plane_tail_color.x, (float)plane_tail_color.y, (float)plane_tail_color.z};
                    
                    glMaterialfv(GL_FRONT, GL_AMBIENT, polygon_color2);
                    glMaterialfv(GL_FRONT, GL_DIFFUSE, box_diffuse);
                    glMaterialfv(GL_FRONT, GL_SPECULAR, box_specular);
                    glMaterialfv(GL_FRONT, GL_SHININESS, box_shininess);
                    
                    glMaterialfv(GL_BACK, GL_AMBIENT, polygon_color2);
                    glMaterialfv(GL_BACK, GL_DIFFUSE, box_diffuse);
                    glMaterialfv(GL_BACK, GL_SPECULAR, box_specular);
                    glMaterialfv(GL_BACK, GL_SHININESS, box_shininess);
                    
                    for(int k = 0; k < 3; k++)
                    {
                        vertices[k] = flock_simulators[0].particles[j].transform_point(paper_plane[1].vertices[k]);
                        glVertex3f((float)vertices[k].x, (float)vertices[k].y, (float)vertices[k].z);
                    }
                    glEnd();
                    
                    glBegin(GL_POLYGON);
                    for(int k = 2; k >= 0; k--)
                    {
                        glVertex3f((float)vertices[k].x, (float)vertices[k].y, (float)vertices[k].z);
                    }
                    glEnd();
                }
            }
        }
        
        //draw the sphere obstacles
        for (int i = 0; i < flock_simulators[0].obstacles.size(); i++)
        {
            //GLfloat sphere_ambient[] = {(float)0.7, (float)0.8, (float)0.8};
            //GLfloat sphere_diffuse[] = {(float)0.7, (float)0.8, (float)0.8};
            
            GLfloat sphere_ambient[] = {(float)col0gb.x, (float)col0gb.y, (float)col0gb.z};
            
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, sphere_ambient);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, sphere_ambient);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
            
            Vector3 sphere_pos = flock_simulators[0].obstacles[i]->get_position();
            float radius = flock_simulators[0].obstacles[i]->get_radius();
            
            glPushMatrix();
                glTranslatef(sphere_pos.x, sphere_pos.y, sphere_pos.z);
                glutSolidSphere(radius, radius*2, radius*2);
            glPopMatrix();
        }
    }
    
    
    //draw the springy meshes
    if(springy_mesh_simulators.size() > 0)
    {
        for(int j = 0; j < springy_mesh_simulators[0].meshes.size(); j++)
        {
            Vector3 line_color = springy_mesh_simulators[0].meshes[j].color;
            
            for(int k = 0; k < springy_mesh_simulators[0].meshes[j].struts.size(); k++)
            {
                int start_point_index = springy_mesh_simulators[0].meshes[j].struts[k].v1;
                int end_point_index   = springy_mesh_simulators[0].meshes[j].struts[k].v2;
                
                Vector3 start_point = springy_mesh_simulators[0].meshes[j].S.elements[start_point_index];
                Vector3 end_point = springy_mesh_simulators[0].meshes[j].S.elements[end_point_index];
                
                //start_point.print();
                
                GLfloat line_diffuse[] = {(float)line_color.x, (float)line_color.y, (float)line_color.z};
                
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, line_diffuse);
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, line_diffuse);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
                
                glBegin(GL_LINES);
                    glVertex3f(start_point.x, start_point.y, start_point.z);
                    glVertex3f(end_point.x, end_point.y, end_point.z);
                glEnd();
            }
        }
    }
    
    
    //draw the rigid bodies
    if(rigid_body_simulators.size() > 0)
    {
        if(write_frame_id % 200 == 0)
            std:cout << "write_frame_id: " << write_frame_id << "\n";
        
        for(int j = 0; j < rigid_body_simulators[0].rigid_bodies.size(); j++)
        {
            Vector3 line_color = rigid_body_simulators[0].rigid_bodies[j].color;
            
            if(write_to_file && write_frame_id < write_frame_count && j > 0)
            {
                add_to_output_rigid_body_simulation(j, rigid_body_simulators[0].rigid_bodies[j].S.pos, rigid_body_simulators[0].rigid_bodies[j].S.q, rigid_body_simulators[0].rigid_bodies[j].r);
            }
            
            if (rigid_body_simulators[0].rigid_bodies[j].render_wireframe)
            {
                for(int k = 0; k < rigid_body_simulators[0].rigid_bodies[j].struts.size(); k++)
                {
                    int start_point_index = rigid_body_simulators[0].rigid_bodies[j].struts[k].v1;
                    int end_point_index   = rigid_body_simulators[0].rigid_bodies[j].struts[k].v2;
                    
                    Vector3 start_point = rigid_body_simulators[0].rigid_bodies[j].vertices[start_point_index].pos;
                    Vector3 end_point   = rigid_body_simulators[0].rigid_bodies[j].vertices[end_point_index].pos;
                    
                    GLfloat line_diffuse[] = {(float)line_color.x, (float)line_color.y, (float)line_color.z};
                    
                    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, line_diffuse);
                    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, line_diffuse);
                    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
                    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
                    
                    glBegin(GL_LINES);
                    glVertex3f(start_point.x, start_point.y, start_point.z);
                    glVertex3f(end_point.x, end_point.y, end_point.z);
                    glEnd();
                }
            }
        }
    }

    if (write_to_file)
    {
        write_frame_id++;
        
        if(write_frame_id == write_frame_count)
        {
            export_file();
        }
    }
    
    //draw the balls
    
    for (int i = 0; i < ball_simulators.size(); i++)
    {
        Vector3 color = ball_simulators[i].ambient_color;
        GLfloat ball_ambient[] = {(float)color.x, (float)color.y, (float)color.z};
        
        color = ball_simulators[i].diffuse_color;
        GLfloat ball_diffuse[] = {(float)color.x, (float)color.y, (float)color.z};
        
        
        Vector3 ball_pos = ball_simulators[i].pos;
        float radius = ball_simulators[i].radius;
        
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ball_ambient);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, ball_diffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ball_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, ball_shininess);
        
        glPushMatrix();
        glTranslatef(ball_pos.x, ball_pos.y, ball_pos.z);
        glutSolidSphere(radius, radius*2, radius*2);
        glPopMatrix();
        
        //draw the trail balls
        if(trails)
        {
            float local_radius = 5;
            
            for(int j = 0; j < 10 ; j++)
            {
                glPushMatrix();
                Vector3 trail_pos = ball_simulators[i].trail_positions[j];
                glTranslatef(trail_pos.x, trail_pos.y, trail_pos.z);
                glutSolidSphere(local_radius, 10, 10);
                glPopMatrix();
                
                local_radius -= 0.4;
            }
        }
    }
    
    glPopMatrix();
    glutSwapBuffers();
}

void update_display_timer(void)
{
    if(simulate)
    {
        if((((double)clock()-startTime) / (CLOCKS_PER_SEC/60)) >= frameDuration)
        {
            startTime  = clock();
            
            display();
            
            for (int i = 0; i < ball_simulators.size(); i++)
                ball_simulators[i].update_simulation(frameDuration);
            
            if(play_from_file)
            {
                if(particle_simulators.size() > 0)
                {
                    particle_simulators[0].update_generator_transform(frames[frame_id].pos * 100 - Vector3(0,100,0), frames[frame_id].normal);
                    generate_particles = frames[frame_id].generate;
                }
                
                if(rigid_body_simulators.size() > 0 && frame_id % 30 == 0 && rigid_body_simulators[0].rigid_bodies.size() < 20)
                {
                    rigid_body_simulators[0].generate_cubes(frames[frame_id].pos * 100 - Vector3(0,100,0), frames[frame_id].normal);
                }
                
                frame_id++;
                
                if (frame_id == frame_count)
                    frame_id = 0;
            }
            
            for (int i = 0; i < particle_simulators.size(); i++)
            {
                particle_simulators[i].update_simulation(frameDuration, generate_particles);
            }
            
            for (int i = 0; i < SPH_simulators.size(); i++)
            {
                SPH_simulators[i].update_simulation(frameDuration, generate_particles);
            }
            
            for (int i = 0; i < flock_simulators.size(); i++)
            {
                flock_simulators[i].update_simulation(frameDuration, generate_particles);
            }
            
            for (int i = 0; i < springy_mesh_simulators.size(); i++)
            {
                springy_mesh_simulators[i].update_simulation(frameDuration);
            }
            
            for (int i = 0; i < rigid_body_simulators.size(); i++)
            {
                rigid_body_simulators[i].update_simulation(frameDuration);
            }
        }
    }
}

void read_all_the_frames()
{
    std::string file_name_2 = "Inputs/RecordData.json";
    std::ifstream file_stream_2(file_name_2);
    
    std::cout << "read_all_the_frames\n";
    srand(10);
    
     if (file_stream_2.is_open())
     {
         json j;
         file_stream_2 >> j;
         frame_count = j["Root"].size();
         
         //frame_count = 500;
         
         frames = new Frame[frame_count];
         
         for (int i = 0; i < frame_count; i++)
         {
             frames[i].pos.x = j["Root"][i]["position_x"];
             frames[i].pos.y = j["Root"][i]["position_y"];
             frames[i].pos.z = j["Root"][i]["position_z"];
             
             frames[i].normal.x = j["Root"][i]["forward_x"];
             frames[i].normal.y = j["Root"][i]["forward_y"];
             frames[i].normal.z = j["Root"][i]["forward_z"];
             
             if(j["Root"][i]["pressed"] == 1)
                 frames[i].generate = true;
         }
         std::cout << j["Root"].size();
     }
}

void read_input_and_initialize(const char* file_suffix)
{
    //std::string file_name = "Inputs/ParticleSimulation" + std::string(file_suffix) + ".json";
    std::string file_name = "Inputs/SPHSimulation" + std::string(file_suffix) + ".json";
    //std::string file_name = "Inputs/FlockSimulation" + std::string(file_suffix) + ".json";
    //std::string file_name = "Inputs/SpringyMeshSimulation" + std::string(file_suffix) + ".json";
    //std::string file_name = "Inputs/RigidBodySimulation" + std::string(file_suffix) + ".json";
    
    // Open file for reading
    std::ifstream file_stream(file_name);
    
    if (file_stream.is_open())
    {
        json j;
        file_stream >> j;
        
        //Create the container
        
        std::string container = j["container"];
        
        if(container.compare("cube") == 0)
            create_cube();
        else if(container.compare("hexagon") == 0)
            create_hexagon_3d();
        else if(container.compare("triangles") == 0)
            create_triangles();
        else if(container.compare("hexagon_with_rectangles") == 0)
            create_hexagon_3d_with_rectangles();
        
        if(j["render_container"] == 1)
            render_container = true;

        if(j["play_from_file"] == 1)
            play_from_file = true;
        
        if(play_from_file)
            read_all_the_frames();

        if(j["write_to_file"] == 1)
            write_to_file = true;
        
        //Read winds
        int wind_count = j["wind_count"];
        
        bool wind_push_and_pull = false;
        int push_and_pull = j["wind_push_and_pull"];
        if (push_and_pull == 1)
            wind_push_and_pull = true;
        
        std::vector<Wind> winds;
        
        for (int i = 0; i < wind_count; i++)
        {
            std::string wind_name = "wind" + std::to_string(i+1);
            
            json source_j = j[wind_name]["source"];
            Vector3 source(source_j[0], source_j[1], source_j[2]);
            
            json velocity_j = j[wind_name]["velocity"];
            Vector3 velocity(velocity_j[0], velocity_j[1], velocity_j[2]);
            
            float angle = j[wind_name]["angle"];
            
            float distance = j[wind_name]["distance"];
            
            winds.push_back(Wind(source, velocity, angle, distance, wind_push_and_pull));
        }
        
        //Add vortex
        int vortex_count = j["vortex_count"];
        if (vortex_count > 0)
        {
            json center_j = j["vortex"]["base_center"];
            Vector3 vortex_center(center_j[0], center_j[1], center_j[2]);
            
            json axis_j = j["vortex"]["axis"];
            Vector3 vortex_axis(axis_j[0], axis_j[1], axis_j[2]);
            
            float vortex_length = j["vortex"]["length"];
            
            float vortex_radius = j["vortex"]["radius"];
            
            float vortex_r_f = j["vortex"]["rotational_frequency"];
            float vortex_max_r_f = j["vortex"]["max_rotational_frequency"];
            float vortex_t = j["vortex"]["vortex_tightness"];
            
            vortex = new Vortex(vortex_center, vortex_axis, vortex_length, vortex_radius, vortex_r_f, vortex_max_r_f, vortex_t);
        }
        
        //Create particle simulator
        int max_particle_count = j["particle_count"];
        
        if (max_particle_count > 0)
        {
            int generator_count = j["particle_generator_count"];
            
            particle_simulators.push_back(ParticleSimulator(max_particle_count));
            particle_simulators[0].vortex = vortex;
            
            for (int i = 0; i < generator_count; i++)
            {
                std::string generator_name = "generator" + std::to_string(i+1);
                
                string type = j[generator_name]["type"];
                
                json pos_j = j[generator_name]["position"];
                Vector3 pos(pos_j[0], pos_j[1], pos_j[2]);
                
                json v_j = j[generator_name]["velocity"];
                Vector3 v(v_j[0], v_j[1], v_j[2]);
                
                float angular_range = j[generator_name]["angular_range"];
                
                float rate = j[generator_name]["rate"];
                
                float m = j[generator_name]["mass"];
                
                float surface_radius = j[generator_name]["surface_radius"];
                
                float air_resistance = j[generator_name]["air_resistance"];
                
                float life_span = j[generator_name]["life_span"];

                json start_color_j = j[generator_name]["start_color"];
                Vector3 start_color(start_color_j[0], start_color_j[1], start_color_j[2]);
                
                json end_color_j = j[generator_name]["end_color"];
                Vector3 end_color(end_color_j[0], end_color_j[1], end_color_j[2]);
                
                particle_simulators[0].add_generator(type, pos, v, angular_range, rate, m, surface_radius, air_resistance, life_span, start_color, end_color);
                particle_simulators[0].polygons = polygons;
                particle_simulators[0].polygon_count = polygon_count;
                particle_simulators[0].winds = winds;
                
                if(play_from_file)
                    particle_simulators[0].g = Vector3(0,-5,0);
            }
        }
        
        //Create SPH simulator
        int max_SPH_particle_count = j["SPH_particle_count"];
        
        if (max_SPH_particle_count > 0)
        {
            int generator_count = j["SPH_particle_generator_count"];
            
            SPH_simulators.push_back(SPHSimulator(max_SPH_particle_count));
            SPH_simulators[0].vortex = vortex;
            
            for (int i = 0; i < generator_count; i++)
            {
                std::string generator_name = "SPH_generator" + std::to_string(i+1);
                
                string type = j[generator_name]["type"];
                
                json pos_j = j[generator_name]["position"];
                Vector3 pos(pos_j[0], pos_j[1], pos_j[2]);
                
                json v_j = j[generator_name]["velocity"];
                Vector3 v(v_j[0], v_j[1], v_j[2]);
                
                float angular_range = j[generator_name]["angular_range"];
                
                float rate = j[generator_name]["rate"];
                
                float m = j[generator_name]["mass"];
                
                float surface_radius = j[generator_name]["surface_radius"];
                
                float air_resistance = j[generator_name]["air_resistance"];
                
                json color_j = j[generator_name]["color"];
                Vector3 SPH_particle_color(color_j[0], color_j[1], color_j[2]);
                
                SPH_simulators[0].add_generator(type, pos, v, angular_range, rate, m, surface_radius, air_resistance, SPH_particle_color);
                SPH_simulators[0].polygons = polygons;
                SPH_simulators[0].polygon_count = polygon_count;
                SPH_simulators[0].winds = winds;
                SPH_simulators[0].initialize_uniform_grid(Vector3(-120, -120, -120), Vector3(240, 240, 240), Vector3(100, 100, 100));
                
                //if(play_from_file)
                //    SPH_simulators[0].g = Vector3(0,-5,0);
            }
        }
        
        //Create flock simulator
        int max_boid_count = j["boid_count"];
        
        if (max_boid_count > 0)
        {
            int boid_generator_count = j["boid_generator_count"];
            
            flock_simulators.push_back(FlockSimulator(max_boid_count));
            
            for (int i = 0; i < boid_generator_count; i++)
            {
                std::string generator_name = "boid_generator" + std::to_string(i+1);
                
                string type = j[generator_name]["type"];
                
                json pos_j = j[generator_name]["position"];
                Vector3 pos(pos_j[0], pos_j[1], pos_j[2]);
                
                json v_j = j[generator_name]["velocity"];
                Vector3 v(v_j[0], v_j[1], v_j[2]);
                
                float angular_range = j[generator_name]["angular_range"];
                
                float rate = j[generator_name]["rate"];
                
                float m = j[generator_name]["mass"];
                
                float surface_radius = j[generator_name]["surface_radius"];
                
                float air_resistance = j[generator_name]["air_resistance"];
                
                float life_span = j[generator_name]["life_span"];
                
                json start_color_j = j[generator_name]["start_color"];
                Vector3 start_color(start_color_j[0], start_color_j[1], start_color_j[2]);
                
                json end_color_j = j[generator_name]["end_color"];
                Vector3 end_color(end_color_j[0], end_color_j[1], end_color_j[2]);
                
                flock_simulators[0].add_generator(type, pos, v, angular_range, rate, m, surface_radius, air_resistance, life_span, start_color, end_color);
                flock_simulators[0].generators[i]->set_particle_type(j[generator_name]["particle_type"], j[generator_name]["polygon_type"]);
                
                json g = j["gravity"];
                flock_simulators[0].g = Vector3(g[0], g[1], g[2]);
                
                if(play_from_file)
                    flock_simulators[0].g = Vector3(0,-5,0);
            }
            
            flock_simulators[0].ka = j["boid_constants"]["ka"];
            flock_simulators[0].kv = j["boid_constants"]["kv"];
            flock_simulators[0].kc = j["boid_constants"]["kc"];
            
            flock_simulators[0].r1 = j["boid_constants"]["distance_threshold_1"];
            flock_simulators[0].r2 = j["boid_constants"]["distance_threshold_2"];
            
            flock_simulators[0].theta1 = j["boid_constants"]["angle_threshold_frontal_binocular"];
            flock_simulators[0].theta1 *= TO_RADIANS_MULTIPLIER * 0.5;
            flock_simulators[0].theta2 = j["boid_constants"]["angle_threshold_peripheral_monocular"];
            flock_simulators[0].theta2 *= TO_RADIANS_MULTIPLIER * 0.5;
            
            flock_simulators[0].k_banking = j["boid_constants"]["k_banking"];
            flock_simulators[0].banking_smoothing_constant = j["boid_constants"]["banking_smoothing_constant"];
            
            flock_simulators[0].polygons = polygons;
            flock_simulators[0].polygon_count = polygon_count;
            flock_simulators[0].winds = winds;
            flock_simulators[0].vortex = vortex;
            

            string lead_boid_controller = j["lead_boid_behavior"]["controller"];
            
            if(lead_boid_controller.compare("physics") == 0)
                flock_simulators[0].lead_boid_controlled_by_physics = true;
            else if(lead_boid_controller.compare("sine") == 0)
            {
                flock_simulators[0].lead_boid_controlled_by_sine_function = true;
                json p = j["lead_boid_behavior"]["position"];
                flock_simulators[0].lead_boid_start_position = Vector3(p[0], p[1], p[2]);
                flock_simulators[0].lead_boid_speed = j["lead_boid_behavior"]["speed"];
            }
            
            
            int obstacle_count = j["obstacle_count"];
            
            for (int i = 0; i < obstacle_count; i++)
            {
                std::string obstacle_name = "obstacle" + std::to_string(i+1);
                
                string type = j[obstacle_name]["type"];
                
                json center_j = j[obstacle_name]["center"];
                Vector3 center(center_j[0], center_j[1], center_j[2]);
                
                float radius = j[obstacle_name]["radius"];
                
                float safe_distance = j[obstacle_name]["safe_distance"];
                
                float threshold_time = j[obstacle_name]["threshold_time"];
                
                flock_simulators[0].add_obstacle(type, center, radius, safe_distance, threshold_time);
            }
        }
        
        //Create springy mesh simulator

        int springy_mesh_count = j["springy_mesh_count"];
        
        if (springy_mesh_count > 0)
        {
            std::string integration_method = j["integration"];
        springy_mesh_simulators.push_back(SpringyMeshSimulator(integration_method));
           
            for (int i = 0; i < springy_mesh_count; i++)
            {
                std::string springy_mesh_name = "springy_mesh" + std::to_string(i+1);
                
                std::string type = j[springy_mesh_name]["type"];
                
                if(type.compare("cube") == 0)
                {
                    json pos_j = j[springy_mesh_name]["initial_position"];
                    Vector3 pos(pos_j[0], pos_j[1], pos_j[2]);
                    
                    json vel_j = j[springy_mesh_name]["initial_velocity"];
                    Vector3 velocity(vel_j[0], vel_j[1], vel_j[2]);
                    
                    float radius = j[springy_mesh_name]["radius"];
                    float mass = j[springy_mesh_name]["mass"];
                    
                    float time_constant = j[springy_mesh_name]["time_constant"];
                    float period_of_oscillation = j[springy_mesh_name]["period_of_oscillation"];
                    
                    float drag_coefficient = j[springy_mesh_name]["drag_coefficient"];
                    float lift_coefficient = j[springy_mesh_name]["lift_coefficient"];
                    
                    json color_j = j[springy_mesh_name]["color"];
                    Vector3 mesh_color(color_j[0], color_j[1], color_j[2]);
                    
                    std::string render_type = j[springy_mesh_name]["render_type"];
                    
                    springy_mesh_simulators[0].add_cube_springy_mesh(pos, velocity, radius, mass, time_constant, period_of_oscillation, drag_coefficient, lift_coefficient, mesh_color, render_type);
                }
            }

            json g = j["gravity"];
            springy_mesh_simulators[0].g = Vector3(g[0], g[1], g[2]);
            
            springy_mesh_simulators[0].polygons = polygons;
            springy_mesh_simulators[0].polygon_count = polygon_count;
            springy_mesh_simulators[0].winds = winds;
            springy_mesh_simulators[0].vortex = vortex;
            springy_mesh_simulators[0].prepare_edge_collision_list();
            double h_denom = j["h_denom"];
            springy_mesh_simulators[0].h = (double)1/h_denom;
        }
        
        //Create rigid body simulator
        
        int rigid_body_count = j["rigid_body_count"];
        
        if (rigid_body_count > 0)
        {
            std::string integration_method = j["integration"];
            rigid_body_simulators.push_back(RigidBodySimulator(integration_method));
            
            for (int i = 0; i < rigid_body_count; i++)
            {
                std::string rigid_body_name = "rigid_body" + std::to_string(i+1);
                
                std::string type = j[rigid_body_name]["type"];
                
                if(type.compare("cube") == 0)
                {
                    json pos_j = j[rigid_body_name]["initial_position"];
                    Vector3 pos(pos_j[0], pos_j[1], pos_j[2]);
                    
                    json rot_j = j[rigid_body_name]["initial_rotation"];
                    Vector3 rot(rot_j[0], rot_j[1], rot_j[2]);
                    
                    json vel_j = j[rigid_body_name]["initial_velocity"];
                    Vector3 velocity(vel_j[0], vel_j[1], vel_j[2]);
                    
                    json angular_vel_j = j[rigid_body_name]["initial_angular_velocity"];
                    Vector3 angular_velocity(angular_vel_j[0], angular_vel_j[1], angular_vel_j[2]);
                    
                    float radius = j[rigid_body_name]["radius"];
                    float mass = j[rigid_body_name]["mass"];
                    
                    float drag_coefficient = j[rigid_body_name]["drag_coefficient"];
                    float lift_coefficient = j[rigid_body_name]["lift_coefficient"];
                    
                    json color_j = j[rigid_body_name]["color"];
                    Vector3 mesh_color(color_j[0], color_j[1], color_j[2]);
                    
                    std::string render_type = j[rigid_body_name]["render_type"];
                    
                    rigid_body_simulators[0].add_cube_rigid_body(pos, rot, velocity, angular_velocity, radius, mass, drag_coefficient, lift_coefficient, mesh_color, render_type);
                }
            }
            
            json g = j["gravity"];
            rigid_body_simulators[0].g = Vector3(g[0], g[1], g[2]);
            
            rigid_body_simulators[0].polygons = polygons;
            rigid_body_simulators[0].polygon_count = polygon_count;
            rigid_body_simulators[0].winds = winds;
            rigid_body_simulators[0].vortex = vortex;
            rigid_body_simulators[0].prepare_edge_collision_list();
            double h_denom = j["h_denom"];
            rigid_body_simulators[0].h = (double)1/h_denom;
        }
        
        //Create ball simulators
        
        int ball_count = j["ball_count"];
        
        if (ball_count > 0)
        {
            int show_trails = j["ball_trail"];
            
            if(show_trails == 1)
                trails = true;
            
            for (int i = 0; i < ball_count; i++)
            {
                std::string ball_name = "ball" + std::to_string(i+1);
                
                json pos_j = j[ball_name]["position"];
                Vector3 pos(pos_j[0], pos_j[1], pos_j[2]);
                
                json v_j = j[ball_name]["velocity"];
                Vector3 v(v_j[0], v_j[1], v_j[2]);
                
                float m = j[ball_name]["mass"];
                
                float radius = j[ball_name]["radius"];
                
                float d = j[ball_name]["air_resistance"];
                
                ball_simulators.push_back(BallSimulator(pos, v, m, radius, d));
                
                json a_c_j = j[ball_name]["ambient_color"];
                Vector3 a_c(a_c_j[0], a_c_j[1], a_c_j[2]);
                
                ball_simulators[i].ambient_color = a_c;
                
                json d_c_j = j[ball_name]["diffuse_color"];
                Vector3 d_c(d_c_j[0], d_c_j[1], d_c_j[2]);
                
                ball_simulators[i].diffuse_color = d_c;
                
                ball_simulators[i].polygons = polygons;
                ball_simulators[i].polygon_count = polygon_count;
                ball_simulators[i].winds = winds;
                ball_simulators[i].vortex = vortex;
            }
        }
    }
    else
        std::cout << "File not open\n";
    
    file_stream.close();
}

void toggle_trails(void)
{
    trails = !trails;
}

void change_timestep(int modifier)
{
    double local_modifier = 0.01;
    for(int i = 0; i < ball_simulators.size(); i++)
    {
        if (ball_simulators[i].h + local_modifier*modifier > 0.0002)
        {
            ball_simulators[i].h += local_modifier*modifier;
        }
    }
}

void init(char* file_suffix)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    // Enable Z-buffering, backface culling, and lighting
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT3);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0, 1, 1500);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Set eye point and lookat point
    gluLookAt(0, 225, 300, 0, 0, 0, 0, 1, 0);
    
    // Set up lights
    GLfloat light0color[] = { 1.0, 1.0, 1.0 };
    GLfloat light0pos[] = { 0, 100, -300 }; //{ 0, 500, 300};
    GLfloat light1color[] = { 1.0, 1.0, 1.0 };
    GLfloat light1pos[] = {0, 100, -300 }; //{ 300, 300, 300 };
    GLfloat light2color[] = { 1.0, 1.0, 1.0 };
    GLfloat light2pos[] = {0, 100, -300 }; //{ 300, 300, 300 };
    glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0color);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0color);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light0color);
    glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light1color);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1color);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light1color);
    glLightfv(GL_LIGHT2, GL_POSITION, light2pos);
    glLightfv(GL_LIGHT2, GL_AMBIENT, light2color);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, light2color);
    glLightfv(GL_LIGHT2, GL_SPECULAR, light2color);
    
    read_input_and_initialize(file_suffix);
    create_paper_plane();
    
    StartTimer();
}

void reshapeFunc(GLint newWidth, GLint newHeight)
{
    if (newWidth > newHeight) // Keep a square viewport
        glViewport((newWidth - newHeight) / 2, 0, newHeight, newHeight);
    else
        glViewport(0, (newHeight - newWidth) / 2, newWidth, newWidth);
    //init();
    glutPostRedisplay();
}

void rotateview(void)
{
    if (rotateon) {
        spin += xchange / 250.0;
        if (spin >= 360.0) spin -= 360.0;
        if (spin < 0.0) spin += 360.0;
        spinup -= ychange / 250.0;
        if (spinup > 89.0) spinup = 89.0;
        if (spinup < -89.0) spinup = -89.0;
    }
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
    switch (button) {
        case GLUT_LEFT_BUTTON:
            if (state == GLUT_DOWN) {
                lastx = x;
                lasty = y;
                xchange = 0;
                ychange = 0;
                rotateon = 1;
            }
            else if (state == GLUT_UP) {
                xchange = 0;
                ychange = 0;
                rotateon = 0;
            }
            break;
            
        default:
            break;
    }
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 't':
            toggle_trails();
            break;
            
        case SPACEBAR:
            if (simulate == false)
                StartTimer();
            break;
            
        case 'w':
            change_timestep(1);
            break;
            
        case 's':
            change_timestep(-1);
            break;
            
        case 'd':
            rigid_body_simulators[0].h = (double)1/90;
            break;
            
        default:
            break;
    }
}

void motion(int x, int y)
{
    xchange = x - lastx;
    ychange = y - lasty;
}

int main(int argc, char** argv)
{
    GLint SubMenu1, SubMenu2, SubMenu3, SubMenu4;
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1000, 1000);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("SPH");
    init(argv[1]);
    rotateon = 0;
    glutDisplayFunc(update_display_timer);
    glutMouseFunc(mouse);
    glutKeyboardFunc(keyboard);
    glutMotionFunc(motion);
    glutIdleFunc(rotateview);
    glutReshapeFunc(reshapeFunc);
    
    glutMainLoop();
    
    return 0;
}
