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
#include "TRIANGLE.h"
#include <vector>

#define VOXEL_SIZE 0.05

using namespace SimLib;
using namespace std;
typedef VECTOR<float, 3> TV;
typedef VECTOR<int, 3> TV_INT;

Rigid_Geometry* g_other = 0;

Rigid_Geometry::Rigid_Geometry()
: implicit_object(grid, phi), kf(1)
{
    show_levelset = false;
    rotation = Quaternion<double>::fromEulerAngles(0, 0, 0);
}

Rigid_Geometry::Rigid_Geometry(const char* _name, const char* filename, const Vector3d &_x, double _m, bool showLevelSet)
: implicit_object(grid, phi), kf(0.3)
{
    show_levelset = showLevelSet;
    name = string(_name);
    rotation = Quaternion<double>::fromEulerAngles(0, 0, 0);
    x = _x;
    mass = _m;
    scale = Vector3d(1, 1, 1);
    triangles.loadOBJ(filename);
    triangles.Update_Bounding_Box_And_Gravity_Center();
    gravity_center = triangles.gravity_center;
    for (int i = 0; i < 3; ++i) {
        J.at(i,i) = 0;
    }
    for (vector<TV>::iterator it = triangles.vertices.begin();
         it != triangles.vertices.end(); ++it) {
        J.at(0,0) += ((*it)(2) - gravity_center(2))*((*it)(2) - gravity_center(2))+((*it)(3) - gravity_center(3))*((*it)(3) - gravity_center(3));
        J.at(1,1) += ((*it)(1) - gravity_center(1))*((*it)(1) - gravity_center(1))+((*it)(3) - gravity_center(3))*((*it)(3) - gravity_center(3));
        J.at(2,2) += ((*it)(2) - gravity_center(2))*((*it)(2) - gravity_center(2))+((*it)(1) - gravity_center(1))*((*it)(1) - gravity_center(1));
        double a01 = -((*it)(1) - gravity_center(1))*((*it)(2) - gravity_center(2));
        double a02 = -((*it)(1) - gravity_center(1))*((*it)(3) - gravity_center(3));
        double a12 = -((*it)(3) - gravity_center(3))*((*it)(2) - gravity_center(2));
        J.at(0,1) = a01;
        J.at(1,0) = a01;
        J.at(0,2) = a02;
        J.at(2,0) = a02;
        J.at(1,2) = a12;
        J.at(2,1) = a12;
    }
    J = (J * (3 * mass / triangles.vertices.size())).inverse();
    RANGE<TV> range = triangles.bounding_box;
    range.min -= TV(0.2, 0.2, 0.2);
    range.max += TV(0.2, 0.2, 0.2);
    grid = GRID<TV>(TV_INT((int)(range.X() / VOXEL_SIZE),(int)(range.Y() / VOXEL_SIZE),(int)(range.Z() / VOXEL_SIZE)), range);
    phi = ARRAY<3, float>(grid.Domain_Indices());
    SimLib::LEVELSET_MAKER<float> level_maker;
    level_maker.Compute_Level_Set(triangles, grid, phi);
    implicit_object.Fast_Marching_Method();
}

Rigid_Geometry::Rigid_Geometry(const char* _name, const char* filename, const Vector3d &_x, const Vector3d &_v, double _m, bool showLevelSet)
: implicit_object(grid, phi), kf(0.3)
{
    scale = Vector3d(1, 1, 1);
    show_levelset = showLevelSet;
    name = string(_name);
    rotation = Quaternion<double>::fromEulerAngles(0, 45, 0);
    x = _x;
    v = _v;
    mass = _m;
    triangles.loadOBJ(filename);
    triangles.Update_Bounding_Box_And_Gravity_Center();
    gravity_center = triangles.gravity_center;
    for (int i = 0; i < 3; ++i) {
        J0.at(i,i) = 0;
    }
    for (vector<TV>::iterator it = triangles.vertices.begin();
         it != triangles.vertices.end(); ++it) {
        J0.at(0,0) += ((*it)(2) - gravity_center(2))*((*it)(2) - gravity_center(2))+((*it)(3) - gravity_center(3))*((*it)(3) - gravity_center(3));
        J0.at(1,1) += ((*it)(1) - gravity_center(1))*((*it)(1) - gravity_center(1))+((*it)(3) - gravity_center(3))*((*it)(3) - gravity_center(3));
        J0.at(2,2) += ((*it)(2) - gravity_center(2))*((*it)(2) - gravity_center(2))+((*it)(1) - gravity_center(1))*((*it)(1) - gravity_center(1));
        double a01 = -((*it)(1) - gravity_center(1))*((*it)(2) - gravity_center(2));
        double a02 = -((*it)(1) - gravity_center(1))*((*it)(3) - gravity_center(3));
        double a12 = -((*it)(3) - gravity_center(3))*((*it)(2) - gravity_center(2));
        J0.at(0,1) = a01;
        J0.at(1,0) = a01;
        J0.at(0,2) = a02;
        J0.at(2,0) = a02;
        J0.at(1,2) = a12;
        J0.at(2,1) = a12;
    }
    J0 = (J0 * (mass / triangles.vertices.size()));
    J = J0.inverse();
    RANGE<TV> range = triangles.bounding_box;
    grid = GRID<TV>(TV_INT(min(40,(int)(range.X() / VOXEL_SIZE)), min(40,(int)(range.Y() / VOXEL_SIZE)), min(40,(int)(range.Z() / VOXEL_SIZE))), range);
    phi = ARRAY<3, float>(grid.Domain_Indices());
    SimLib::LEVELSET_MAKER<float> level_maker;
    level_maker.Compute_Level_Set(triangles, grid, phi);
    implicit_object.Fast_Marching_Method();
}

void Rigid_Geometry::Translate(const Vector3d& trans) {
    x += trans;
}

void Rigid_Geometry::Rotate(const Vector3d& rotate) {
    rotation = Quaternion<double>::fromEulerAngles(rotate[0], rotate[1], rotate[2]) * rotation;
}

void Rigid_Geometry::Scale(const Vector3d& s) {
    scale *= s;
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

void Rigid_Geometry::ExcertMoment(const Vector3d &force, const Vector3d &p) {
    if (!nailed) {
        M += (p - x).crossProduct(force);
    }
}

void Rigid_Geometry::ExcertMoment(const Vector3d &moment) {
    if (!nailed) {
        M += moment;
    }
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
//    glDisable(GL_LIGHTING);
    glPushMatrix();
    glTranslated(x[0], x[1], x[2]);
    glMultMatrixd(rotation.transform().data);
    glScaled(scale[0], scale[1], scale[2]);
    glColor3d(1, 0, 0);
    if (!show_levelset) {
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < triangles.triangle_list.size(); ++i) {
            glNormal3d(triangles.triangle_list[i].n1(1),triangles.triangle_list[i].n1(2),triangles.triangle_list[i].n1(3));
            glVertex3d(triangles.triangle_list[i].a(1),triangles.triangle_list[i].a(2),triangles.triangle_list[i].a(3));
            glNormal3d(triangles.triangle_list[i].n2(1),triangles.triangle_list[i].n2(2),triangles.triangle_list[i].n2(3));
            glVertex3d(triangles.triangle_list[i].b(1),triangles.triangle_list[i].b(2),triangles.triangle_list[i].b(3));
            glNormal3d(triangles.triangle_list[i].n3(1),triangles.triangle_list[i].n3(2),triangles.triangle_list[i].n3(3));
            glVertex3d(triangles.triangle_list[i].c(1),triangles.triangle_list[i].c(2),triangles.triangle_list[i].c(3));
        }
        glEnd();
    } else {
        glColor3d(0,0,1);
        for (int i = 1; i <= phi.dim(1); ++i) {
            for (int j = 1; j <= phi.dim(2); ++j) {
                for (int k = 1; k <= phi.dim(3); ++k) {
                    if (phi(i,j,k) < 0) {
                        glColor3d(0,0,-phi(i,j,k));
                        TV t = grid.dom_min + TV(i,j,k) * grid.dx;
                        glPushMatrix();
                        glTranslated(t(1), t(2), t(3));
                        glutSolidCube(0.01);
                        glPopMatrix();
                    }
                }
            }
        }
    }
    glPopMatrix();
    glPushMatrix();
    glTranslated(x[0], x[1], x[2]);
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

Vector3d Rigid_Geometry::ApplyTransform(const TV& p) {
    Vector3f v(p(1) - gravity_center(1), p(2) - gravity_center(2), p(3) - gravity_center(3));
    v *= scale;
    v = rotation.rotMatrix() * v;
    return Vector3d(v[0] + x[0], v[1] + x[1], v[2] + x[2]);
}

Matrix4d Rigid_Geometry::Transform() const {
    return Matrix4d::createTranslation(x[0], x[1], x[2]) * rotation.transform() * Matrix4d::createScale(scale[0], scale[1], scale[2]) * Matrix4d::createTranslation(-gravity_center(1), -gravity_center(2), -gravity_center(3));
}

Matrix4d Rigid_Geometry::Inv_Transform() const {
    Quaternion<double> q = rotation;
    q.w = -q.w;
    return Matrix4d::createTranslation(gravity_center(1), gravity_center(2), gravity_center(3)) * Matrix4d::createScale(1 / scale[0], 1 / scale[1], 1 / scale[2]) * q.transform() * Matrix4d::createTranslation(-x[0], -x[1], -x[2]);
}

void Rigid_Geometry::collid_detection(Geometric* g, std::vector<Contact>* contact) {
    Rigid_Geometry* rgb = dynamic_cast<Rigid_Geometry*>(g);
    if (rgb) {
        if (!triangles.bounding_box.Lazy_Intersection(rgb->triangles.bounding_box))
            return;
        Vector4d o1(gravity_center(1), gravity_center(2), gravity_center(3), 1);
        Vector4d o2(rgb->gravity_center(1),rgb->gravity_center(2),rgb->gravity_center(3), 1);
        Matrix4<float> tr = rgb->Inv_Transform() * Transform();
        pair<float, TV> deepest_intersection(1e30, TV(0,0,0));
        Vector4d cp;
        for (vector<TV>::iterator it = triangles.vertices.begin();
             it != triangles.vertices.end(); ++it) {
            Vector4d p = tr * Vector4d((*it)(1),(*it)(2),(*it)(3),1);
            pair<float, TV> intersecion = rgb->implicit_object.Intersect(TV(p[0], p[1], p[2]));
            if (intersecion.first > 0)
                continue;
            if (contact) {
                Vector3d N = rgb->Transform() * Vector3d(intersecion.second(1),intersecion.second(2),intersecion.second(3));
                Vector4d pp = rgb->Transform() * p;
                Vector3d r1(pp[0] - x[0], pp[1] - x[1], pp[2] - x[2]);
                Vector3d r2(pp[0] - rgb->x[0], pp[1] - rgb->x[1], pp[2] - rgb->x[2]);
                Vector3d v1 = v + w.crossProduct(r1);
                Vector3d v2 = rgb->v + rgb->w.crossProduct(r2);
                Vector3d V(v2 - v1);
                double collid = N.dotProduct(V);
                if (fabs(collid) < CONTACT_THRESHOLD) {
                    Contact data;
                    data.a = this;
                    data.b = rgb;
                    data.n = N;
                    data.p = Vector3d(pp[0],pp[1],pp[2]);
                    data.ra = r1;
                    data.rb = r2;
                    contact->push_back(data);
                }
            }
            if (intersecion.first < deepest_intersection.first) {
                cp = p;
                deepest_intersection = intersecion;
            }
        }
        
        if (deepest_intersection.first > 0)
            return;
        cp = rgb->Transform() * cp;
        Vector3d N = rgb->Transform() * Vector3d(deepest_intersection.second(1),deepest_intersection.second(2),deepest_intersection.second(3));
        N.normalize();
        Vector3d r1(cp[0] - x[0], cp[1] - x[1], cp[2] - x[2]);
        Vector3d r2(cp[0] - rgb->x[0], cp[1] - rgb->x[1], cp[2] - rgb->x[2]);
        Vector3d v1 = v + w.crossProduct(r1);
        Vector3d v2 = rgb->v + rgb->w.crossProduct(r2);
        Vector3d V(v2 - v1);
        double collid = N.dotProduct(V);

        if (collid > EPSILON){
            Matrix3d rot1 = rotation.rotMatrix();
            Matrix3d J1 = rot1 * J * rot1.transpose();
            Matrix3d rot2 = rgb->rotation.rotMatrix();
            Matrix3d J2 = rot2 * rgb->J * rot2.transpose();
            double m1 = 1 / mass;
            double m2 = 1 / rgb->mass;
            if (nailed) {
                m1 = 0;
                J1 = Matrix3d::createScale(0, 0, 0);
            }
            if (rgb->nailed) {
                m2 = 0;
                J2 = Matrix3d::createScale(0, 0, 0);
            }
            Matrix3d crossM1 = Matrix3d::createCrossProductMatrix(r1);
            Matrix3d crossM2 = Matrix3d::createCrossProductMatrix(r2);
            Matrix3d K = Matrix3d() * (m1 + m2) + crossM1.transpose() * J1 * crossM1 + crossM2.transpose() * J2 * crossM2;
            double kc = 0;
            if (collid > CONTACT_THRESHOLD) {
                kc = max(kr, rgb->kr);
            }
            double jn = (1 + kc) * V.dotProduct(N) / N.dotProduct(K * N);
            v += N * (jn * m1);
            rgb->v -= N * (jn * m2);
            w += J1 * (r1.crossProduct(N) * jn);
            rgb->w -= J2 * (r1.crossProduct(N) * jn);
            
/*            Vector3d Vt = V - N * V.dotProduct(N);
            if (Vt.length() < 1e-4)
                return;
            Vector3d Vtn = Vt;
            Vtn.normalize();
            double jt = Vt.length() / Vtn.dotProduct(K * Vtn);
            if (jt > jn * max(kf, rgb->kf)) {
                jt = jn * max(kf, rgb->kf);
            }
            v += Vtn * (jt * m1);
            rgb->v -= Vtn * (jt * m2);
            w += J1 * (r1.crossProduct(Vtn) * jt);
            rgb->w -= J2 * (r2.crossProduct(Vtn) * jt);
*/        }
    }
}
