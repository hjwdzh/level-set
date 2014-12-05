//
//  Rigid_Geometry.cpp
//  levelset
//
//  Created by Jingwei Huang on 14-10-6.
//  Copyright (c) 2014年 Jingwei Huang. All rights reserved.
//

#include "Rigid_Geometry.h"
#include "main.h"
#include "Particle.h"
#include "vmath.h"
#include "LEVELSET_MAKER.h"
#include <vector>

#define VOXEL_SIZE 0.05

using namespace SimLib;
using namespace std;
typedef VECTOR<float, 3> TV;
typedef VECTOR<int, 3> TV_INT;

Rigid_Geometry* g_other = 0;

Rigid_Geometry::Rigid_Geometry()
: implicit_object(grid, phi)
{
    rotation = Quaternion<double>::fromEulerAngles(0, 45, 0);
}

Rigid_Geometry::Rigid_Geometry(char* _name, char* filename, const Vector3d &_x, double _m)
: implicit_object(grid, phi)
{
    name = string(_name);
    rotation = Quaternion<double>::fromEulerAngles(0, 45, 0);
    x = _x;
    mass = _m;
    triangles.loadOBJ(filename);
    triangles.Update_Bounding_Box_And_Gravity_Center();
    gravity_center = triangles.gravity_center;
    RANGE<TV> range = triangles.bounding_box;
    range.min -= TV(0.2, 0.2, 0.2);
    range.max += TV(0.2, 0.2, 0.2);
    grid = GRID<TV>(TV_INT((int)(range.X() / VOXEL_SIZE),(int)(range.Y() / VOXEL_SIZE),(int)(range.Z() / VOXEL_SIZE)), range);
    phi = ARRAY<3, float>(grid.Domain_Indices());
    SimLib::LEVELSET_MAKER<float> level_maker;
    level_maker.Compute_Level_Set(triangles, grid, phi);
    implicit_object.Fast_Marching_Method();
}

Rigid_Geometry::Rigid_Geometry(char* _name, char* filename, const Vector3d &_x, const Vector3d &_v, double _m)
: implicit_object(grid, phi)
{
    name = string(_name);
    rotation = Quaternion<double>::fromEulerAngles(90, 0, 0);
    x = _x;
    v = _v;
    mass = _m;
    triangles.loadOBJ(filename);
    triangles.Update_Bounding_Box_And_Gravity_Center();
    gravity_center = triangles.gravity_center;
    RANGE<TV> range = triangles.bounding_box;
    grid = GRID<TV>(TV_INT(min(40,(int)(range.X() / VOXEL_SIZE)), min(40,(int)(range.Y() / VOXEL_SIZE)), min(40,(int)(range.Z() / VOXEL_SIZE))), range);
    phi = ARRAY<3, float>(grid.Domain_Indices());
    SimLib::LEVELSET_MAKER<float> level_maker;
    level_maker.Compute_Level_Set(triangles, grid, phi);
    implicit_object.Fast_Marching_Method();
}

double Rigid_Geometry::getMouseDepth(double mouseX, double mouseY)
{
    Vector4d v(x[0], x[1], x[2], 1);
    v = g_camera->lookat * v;
    v = v / v[3];
    double dx = (v[0] / v[2] * (-g_d) - mouseX) / g_right * g_WindowWidth * 0.5;
    double dy = (v[1] / v[2] * (-g_d) - mouseY) / g_top * g_WindowHeight * 0.5;
    if (sqrt(dx * dx + dy * dy) < 10)
    {
        return v[2];
    }
    return 1;
}

void Rigid_Geometry::ExcertForce(const Vector3d &force)
{
    if (!nailed)
        f += force;
}

void Rigid_Geometry::ExcertForceField(Vector3d (*forcefunc)(Geometric*))
{
    if (!nailed)
        f += forcefunc((Geometric*)this);
}

void Rigid_Geometry::Display()
{
    if (nailed)
        glColor3f(0.0f, 0.0f, 1.0f);
    else
        if (selected)
            glColor3f(1.0f, 0.0f, 1.0f);
        else
            glColor3f(0.0, 1.0f, 0.0f);
    glPushMatrix();
    glDisable(GL_LIGHTING);
//    glTranslated(x[0], x[1], x[2]);
//    glMultMatrixd(rotation.transform().data);
    
//    if (strcmp(name.c_str(), "obj2") == 0 && g_other) {
        glTranslated(x[0], x[1], x[2]);
        glMultMatrixd(rotation.transform().data);
        glColor3d(1, 0, 0);
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < triangles.triangle_list.size(); ++i) {
/*            Vector4d p = tr * Vector4d(triangles.triangle_list[i].a(1),triangles.triangle_list[i].a(2),triangles.triangle_list[i].a(3),1);
            glPushMatrix();
            glTranslated(p[0], p[1], p[2]);
            glutSolidCube(0.05);
            glPopMatrix();
            p = tr * Vector4d(triangles.triangle_list[i].b(1),triangles.triangle_list[i].b(2),triangles.triangle_list[i].b(3),1);
            glPushMatrix();
            glTranslated(p[0], p[1], p[2]);
            glutSolidCube(0.05);
            glPopMatrix();
            p = tr * Vector4d(triangles.triangle_list[i].c(1),triangles.triangle_list[i].c(2),triangles.triangle_list[i].c(3),1);
            glPushMatrix();
            glTranslated(p[0], p[1], p[2]);
            glutSolidCube(0.05);
            glPopMatrix();
*/
            glNormal3d(triangles.triangle_list[i].n1(1),triangles.triangle_list[i].n1(2),triangles.triangle_list[i].n1(3));
            glVertex3d(triangles.triangle_list[i].a(1),triangles.triangle_list[i].a(2),triangles.triangle_list[i].a(3));
            glNormal3d(triangles.triangle_list[i].n2(1),triangles.triangle_list[i].n2(2),triangles.triangle_list[i].n2(3));
            glVertex3d(triangles.triangle_list[i].b(1),triangles.triangle_list[i].b(2),triangles.triangle_list[i].b(3));
            glNormal3d(triangles.triangle_list[i].n3(1),triangles.triangle_list[i].n3(2),triangles.triangle_list[i].n3(3));
            glVertex3d(triangles.triangle_list[i].c(1),triangles.triangle_list[i].c(2),triangles.triangle_list[i].c(3));
        }
        glEnd();
//    }

/*    if (strcmp(name.c_str(), "obj1") == 0) {
        glColor3d(0, 0, 1);
        glBegin(GL_POINT);
        g_other = this;
        for (int i = 1; i <= phi.dim(1); ++i) {
            for (int j = 1; j <= phi.dim(2); ++j) {
                for (int k = 1; k <= phi.dim(3); ++k)
                    if (phi(i,j,k) < 0){
                        glPushMatrix();
                        TV ind = grid.dom_min + TV(i,j,k) * grid.dx;
                        glTranslated(ind(1), ind(2), ind(3));
                        glutSolidCube(0.05);
                        glPopMatrix();
                    }
            }
        }
        glEnd();
    }
*/
    glScaled(0.2, 0.2, 0.2);
    if (extForce.length() > 0)
    {
        glColor3f(1.0f, 0.0, 0.0);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(extForce[0], extForce[1], extForce[2]);
        glEnd();
        glTranslated(extForce[0], extForce[1], extForce[2]);
        glutSolidSphere(0.001f, 15, 15);
        glTranslated(-extForce[0], -extForce[1], -extForce[2]);
    }
    if (userForce.length() > 0)
    {
        glColor3f(1.0f, 1.0, 1.0);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(userForce[0], userForce[1], userForce[2]);
        glEnd();
        glTranslated(userForce[0], userForce[1], userForce[2]);
        glutSolidSphere(0.01f, 15, 15);
    }
    glPopMatrix();
}

Vector3d Rigid_Geometry::ApplyTransform(TV& p) {
    Vector3f v(p(1) - gravity_center(1), p(2) - gravity_center(2), p(3) - gravity_center(3));
    v = rotation.rotMatrix() * v;
    return Vector3d(v[0] + x[0], v[1] + x[1], v[2] + x[2]);
}

Matrix4d Rigid_Geometry::Transform() const {
    return Matrix4d::createTranslation(x[0], x[1], x[2]) * rotation.transform() * Matrix4d::createTranslation(-gravity_center(1), -gravity_center(2), -gravity_center(3));
}

Matrix4d Rigid_Geometry::Inv_Transform() const {
    Quaternion<double> q = rotation;
    q.w = -q.w;
    return Matrix4d::createTranslation(gravity_center(1), gravity_center(2), gravity_center(3)) * q.transform() * Matrix4d::createTranslation(-x[0], -x[1], -x[2]);
}

void Rigid_Geometry::collid_detection(Geometric* g) {
    static int t = 0;
    Rigid_Geometry* rgb = dynamic_cast<Rigid_Geometry*>(g);
    if (rgb) {
        Matrix4<float> tr = rgb->Inv_Transform() * Transform();
        for (vector<TV>::iterator it = triangles.vertices.begin();
             it != triangles.vertices.end(); ++it) {
            Vector4d p = tr * Vector4d((*it)(1),(*it)(2),(*it)(3),1);
            pair<float, TV> intersecion = rgb->implicit_object.Intersect(TV(p[0], p[1], p[2]));
            if (intersecion.first > 0 || t > 0)
                continue;
            t++;
            Vector3d N = rgb->Transform() * Vector3d(intersecion.second(1),intersecion.second(2),intersecion.second(3));
            
            Vector3d V(rgb->v - v);
            double collid = N.dotProduct(V) / V.length();
            if (collid > EPSILON){
                Vector3d vn1 = N * N.dotProduct(v);
                Vector3d vt1 = v - vn1;
                Vector3d vn2 = N * N.dotProduct(rgb->v);
                Vector3d vt2 = rgb->v - vn2;
                v = (vn1 * (mass - rgb->mass * (kr * rgb->kr)) + vn2 * rgb->mass * (1 + kr * rgb->kr)) / (mass + rgb->mass);
                rgb->v = (vn1 - vn2) * (kr * rgb->kr) + v + vt2;
                v += vt1;
            }
        }
    }
}