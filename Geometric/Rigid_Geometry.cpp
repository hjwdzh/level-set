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
#include "texture.h"
#include "TriManager.h"
#include "KDOP.h"
#include <fstream>
#include <vector>

using namespace SimLib;
using namespace std;
typedef VECTOR<float, 3> TV;
typedef VECTOR<int, 3> TV_INT;

Rigid_Geometry* g_other = 0;

Rigid_Geometry::Rigid_Geometry()
: implicit_object(0), kf(1), texturemap(0)
{
    show_levelset = false;
    rotation = Quaternion<double>::fromEulerAngles(0, 0, 0);
    bounding_volume = new KDOP<4,float>();
}

Rigid_Geometry::Rigid_Geometry(const char* _name, const char* filename, const Vector3d &_x, const Vector3d &_r, const Vector3d &_s, double _m, IMPLICIT_OBJECT<float>* object, bool showLevelSet)
{
    kf = (0.3);
    show_levelset = showLevelSet;
    texturemap = 0;
    name = string(_name);
    rotation = Quaternion<double>::fromEulerAngles(_r[0], _r[1], _r[2]);
    mass = _m;
    scale = _s;
    
    TriData data = TriManager::createTriMeshAndLevelSet(filename, object == 0);
    triangles = data.triangles;
    x = data.x;
    grid = data.grid;
    phi = data.phi;
    if (object) {
        implicit_object = object;
    } else {
        implicit_object = data.implicit_object;
    }

    x = scale * x;
    x = rotation.rotMatrix() * x;
    x += _x;
    gravity_center = triangles->gravity_center;
    for (int i = 0; i < 3; ++i) {
        J.at(i,i) = 0;
    }
    for (vector<TV>::iterator it = triangles->vertices.begin();
         it != triangles->vertices.end(); ++it) {
        J.at(0,0) += ((*it)(2) - gravity_center(2))*((*it)(2) - gravity_center(2))*_s[1]*_s[1]+((*it)(3) - gravity_center(3))*((*it)(3) - gravity_center(3))*_s[2]*_s[2];
        J.at(1,1) += ((*it)(1) - gravity_center(1))*((*it)(1) - gravity_center(1))*_s[0]*_s[0]+((*it)(3) - gravity_center(3))*((*it)(3) - gravity_center(3))*_s[2]*_s[2];
        J.at(2,2) += ((*it)(2) - gravity_center(2))*((*it)(2) - gravity_center(2))*_s[1]*_s[1]+((*it)(1) - gravity_center(1))*((*it)(1) - gravity_center(1))*_s[0]*_s[0];
        double a01 = -((*it)(1) - gravity_center(1))*((*it)(2) - gravity_center(2))*_s[0]*_s[1];
        double a02 = -((*it)(1) - gravity_center(1))*((*it)(3) - gravity_center(3))*_s[0]*_s[2];
        double a12 = -((*it)(3) - gravity_center(3))*((*it)(2) - gravity_center(2))*_s[2]*_s[1];
        J.at(0,1) = a01;
        J.at(1,0) = a01;
        J.at(0,2) = a02;
        J.at(2,0) = a02;
        J.at(1,2) = a12;
        J.at(2,1) = a12;
    }
    J0 = (J * (mass / triangles->vertices.size()));
    J = J0.inverse();
    bounding_volume = new KDOP<4,float>();
    bounding_volume->rgd = this;
    updateBoundingVolume();
}

void Rigid_Geometry::LoadTexture(const char* bmp, double _tex_scale) {
    texturemap = TexManager::createRenderTexture(bmp);
    tex_scale = _tex_scale;
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

void Rigid_Geometry::updateBoundingVolume() {
    Matrix4d m = Transform();
    bounding_volume->update(triangles->vertices, &m);
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
    glPushMatrix();
    glTranslated(x[0], x[1], x[2]);
    if (strcmp(name.c_str(), "gate") != 0)
        glRotated(180, 0, 1, 0);
    glMultMatrixd(rotation.transform().data);
    glScaled(scale[0], scale[1], scale[2]);
    glColor3d(0, 0, 1);
    if (!show_levelset) {
        if (texturemap) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texturemap);
        } else {
            glDisable(GL_TEXTURE_2D);
        }
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < triangles->triangle_list.size(); ++i) {
            glTexCoord2d(triangles->triangle_list[i].t1(1)*scale[0]*tex_scale, triangles->triangle_list[i].t1(2)*scale[2]*tex_scale);
            glNormal3d(triangles->triangle_list[i].n1(1),triangles->triangle_list[i].n1(2),triangles->triangle_list[i].n1(3));
            glVertex3d(triangles->triangle_list[i].a(1),triangles->triangle_list[i].a(2),triangles->triangle_list[i].a(3));
            glTexCoord2d(triangles->triangle_list[i].t2(1)*scale[0]*tex_scale, triangles->triangle_list[i].t2(2)*scale[2]*tex_scale);
            glNormal3d(triangles->triangle_list[i].n2(1),triangles->triangle_list[i].n2(2),triangles->triangle_list[i].n2(3));
            glVertex3d(triangles->triangle_list[i].b(1),triangles->triangle_list[i].b(2),triangles->triangle_list[i].b(3));
            glTexCoord2d(triangles->triangle_list[i].t3(1)*scale[0]*tex_scale, triangles->triangle_list[i].t3(2)*scale[2]*tex_scale);
            glNormal3d(triangles->triangle_list[i].n3(1),triangles->triangle_list[i].n3(2),triangles->triangle_list[i].n3(3));
            glVertex3d(triangles->triangle_list[i].c(1),triangles->triangle_list[i].c(2),triangles->triangle_list[i].c(3));
        }
        glEnd();
    } else {
        glDisable(GL_LIGHTING);
        glColor3d(0,0,1);
        for (int i = 1; i <= phi->dim(1); ++i) {
            for (int j = 1; j <= phi->dim(2); ++j) {
                for (int k = 1; k <= phi->dim(3); ++k) {
                    if ((*phi)(i,j,k) < 0) {
                        glColor3d(0,0,-(*phi)(i,j,k));
                        TV t = grid->dom_min + TV(i,j,k) * grid->dx;
                        glPushMatrix();
                        glTranslated(t(1), t(2), t(3));
                        glutSolidCube(0.1);
                        glPopMatrix();
                    }
                }
            }
        }
        glEnable(GL_LIGHTING);
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
    if (strcmp(name.c_str(), g->name.c_str()) == 0)
        return;
    Rigid_Geometry* rgb = dynamic_cast<Rigid_Geometry*>(g);
    if (rgb) {
        if (!bounding_volume->intersect(rgb->bounding_volume))
            return;
        Matrix4d tr = rgb->Inv_Transform() * Transform();
        pair<float, TV> deepest_intersection(1e30, TV(0,0,0));
        Vector4d cp;
        for (vector<TV>::iterator it = triangles->vertices.begin();
             it != triangles->vertices.end(); ++it) {
            Vector4d p = tr * Vector4d((*it)(1),(*it)(2),(*it)(3),1);
            pair<float, TV> intersecion = rgb->implicit_object->Intersect(TV(p[0], p[1], p[2]));
            if (intersecion.first > 0)
                continue;
            if (contact) {
                Vector3d N = rgb->Transform() * Vector3d(intersecion.second(1),intersecion.second(2),intersecion.second(3));
                N.normalize();
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
        if (!nailed && contact && deepest_intersection.first < -0.1) {
            x -= N * deepest_intersection.first / 10;
        }
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
                kc = min(kr, rgb->kr);
            }
            double jn = (1 + kc) * V.dotProduct(N) / N.dotProduct(K * N);
            v += N * (jn * m1);
            rgb->v -= N * (jn * m2);
            w += J1 * (r1.crossProduct(N) * jn);
            rgb->w -= J2 * (r2.crossProduct(N) * jn);
            
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
*/      }
        if (system) {
            system->collid_event(this, rgb);
        }
    }
}
