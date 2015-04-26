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
#include "GJKDetector.h"
#include "IMPLICIT_SPHERE.h"
#include "IMPLICIT_CUBE.h"
#include <fstream>
#include <vector>
#include <sys/time.h>

using namespace SimLib;
using namespace std;
typedef VECTOR<float, 3> TV;
typedef VECTOR<int, 3> TV_INT;
int stopsign = 0;
Rigid_Geometry::Rigid_Geometry()
: implicit_object(0), kf(1), texturemap(0)
{
    show_levelset = false;
    rotation = Quaternion<double>::fromEulerAngles(0, 0, 0);
    bounding_volume = new KDOP<3,float>();
}

Rigid_Geometry::Rigid_Geometry(const char* _name, const char* filename, const Vector3d &_x, const Vector3d &_r, const Vector3d &_s, double _m, IMPLICIT_OBJECT<float>* object, bool showLevelSet)
{
    kf = (0.2);
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
    for (vector<VECTOR<float,3> >::iterator it = triangles->vertices.begin();
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
    bounding_volume = new KDOP<3,float>();
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
    bounding_volume->update(triangles->vertices, 0);
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
    glMultMatrixd(rotation.transform().data);
    glScaled(scale[0], scale[1], scale[2]);
    if (!show_levelset) {
        if (texturemap) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texturemap);
        } else {
            glDisable(GL_TEXTURE_2D);
        }
        if (name[0] == 'W') {
            glEnable(GL_BLEND);
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_LIGHTING);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_CULL_FACE);
            glColor4d(0.5, 0.5, 0.5, 0.3);
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
        if (name[0] == 'W') {
            glDisable(GL_BLEND);
            glEnable(GL_LIGHTING);
            glEnable(GL_TEXTURE_2D);
            glDisable(GL_CULL_FACE);
        }
    } else {
        glDisable(GL_LIGHTING);
        glColor3d(0,0,1);
        for (int i = 1; i <= phi->dim(1); ++i) {
            for (int j = 1; j <= phi->dim(2); ++j) {
                for (int k = 1; k <= phi->dim(3); ++k) {
                    if ((*phi)(i,j,k) < 0) {
                        glColor3d(0,0,-(*phi)(i,j,k));
                        VECTOR<float,3> t = grid->dom_min + VECTOR<float,3>(i,j,k) * grid->dx;
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

Vector3d Rigid_Geometry::ApplyTransform(const VECTOR<float,3>& p) {
    Vector3f v(p(1) - gravity_center(1), p(2) - gravity_center(2), p(3) - gravity_center(3));
    v *= scale;
    v = rotation.rotMatrix() * v;
    return Vector3d(v[0] + x[0], v[1] + x[1], v[2] + x[2]);
}

void Rigid_Geometry::updateTransform() {
    transform = Matrix4d::createTranslation(x[0], x[1], x[2]) * rotation.transform() * Matrix4d::createScale(scale[0], scale[1], scale[2]) * Matrix4d::createTranslation(-gravity_center(1), -gravity_center(2), -gravity_center(3));
    Quaternion<double> q = rotation;
    q.w = -q.w;
    inv_transform = Matrix4d::createTranslation(gravity_center(1), gravity_center(2), gravity_center(3)) * Matrix4d::createScale(1 / scale[0], 1 / scale[1], 1 / scale[2]) * q.transform() * Matrix4d::createTranslation(-x[0], -x[1], -x[2]);
}

void Rigid_Geometry::collide_detection(Geometric* g, std::vector<Contact>* contact) {
    Rigid_Geometry* rgb = dynamic_cast<Rigid_Geometry*>(g);
    timeval tt1, tt2;
    if (rgb) {
        gettimeofday(&tt1, 0);
        IMPLICIT_SPHERE<float> *t1, *t2;
        IMPLICIT_CUBE<float> *t3;
        t1 = dynamic_cast<IMPLICIT_SPHERE<float>* >(rgb->implicit_object);
        t2 = dynamic_cast<IMPLICIT_SPHERE<float>* >(implicit_object);
        if (t1 && t2) {
            Vector3d o1(rgb->x.x + t1->o(1), rgb->x.y + t1->o(2), rgb->x.z + t1->o(3));
            Vector3d o2(x.x + t2->o(1), x.y + t2->o(2), x.z + t2->o(3));
            o2 -= o1;
            double l = o2.length();
            if (l < t1->r + t2->r) {
                if (contact) {
                    Vector3d N = o2 / (l);
                    Vector3d pp = o1 + o2 * (t1->r / l);;
                    //Vector4d pp = rgb->Transform() * ((p2 - p1) * portion + p1);
                    Vector3d r1(pp[0] - x[0], pp[1] - x[1], pp[2] - x[2]);
                    Vector3d r2(pp[0] - rgb->x[0], pp[1] - rgb->x[1], pp[2] - rgb->x[2]);
                    Vector3d v1 = v + w.crossProduct(r1);
                    Vector3d v2 = rgb->v + rgb->w.crossProduct(r2);
                    Vector3d V(v2 - v1);
                    double collid = N.dotProduct(V);
                    if (fabs(collid) > -CONTACT_THRESHOLD) {
                        Contact data;
                        data.a = this;
                        data.b = rgb;
                        data.n = N;
                        data.p = Vector3d(pp[0],pp[1],pp[2]);
                        data.ra = r1;
                        data.rb = r2;
                        data.kr = (kr < rgb->kr) ? kr : rgb->kr;
                        data.mu = (kf < rgb->kf) ? kf : rgb->kf;
                        contact->push_back(data);
                    }
                }
            }
            return;
        }
        if (t1 || t2) {
            Vector4d o;
            Rigid_Geometry *cube, *sphere;
            if (t1) {
                o = inv_transform * Vector4d(rgb->x.x, rgb->x.y, rgb->x.z, 1);
                t3 = dynamic_cast<IMPLICIT_CUBE<float>* >(implicit_object);
                cube = this;
                sphere = rgb;
            } else {
                o = rgb->Inv_Transform() * Vector4d(x.x, x.y, x.z, 1);
                t1 = t2;
                t3 = dynamic_cast<IMPLICIT_CUBE<float>* >(rgb->implicit_object);
                cube = rgb;
                sphere = this;
            }
            Vector4d op = o;
            o.x *= cube->scale[0];
            o.y *= cube->scale[1];
            o.z *= cube->scale[2];
            Vector3d N;
            Vector4d pp;
            if (o.x - t1->r < t3->box.min(1)) {
                if ((o.x + t1->r > t3->box.min(1)) && (o.y + t1->r > t3->box.min(2)) && (o.y - t1->r < t3->box.max(2)) && (o.z + t1->r > t3->box.min(3)) && (o.z - t1->r < t3->box.max(3))) {
                    N = Vector3d(-1, 0, 0);
                    pp = op + Vector4d(t1->r, 0, 0, 0);
                }
            } else
            if (o.x + t1->r > t3->box.max(1)) {
                if ((o.x - t1->r < t3->box.max(1)) && (o.y + t1->r > t3->box.min(2)) && (o.y - t1->r < t3->box.max(2)) && (o.z + t1->r > t3->box.min(3)) && (o.z - t1->r < t3->box.max(3))) {
                    N = Vector3d(1, 0, 0);
                    pp = op - Vector4d(t1->r, 0, 0, 0);
                }
            } else
            if (o.y - t1->r < t3->box.min(2)) {
                if ((o.y + t1->r > t3->box.min(2)) && (o.x + t1->r > t3->box.min(1)) && (o.x - t1->r < t3->box.max(1)) && (o.z + t1->r > t3->box.min(3)) && (o.z - t1->r < t3->box.max(3))) {
                    N = Vector3d(0, -1, 0);
                    pp = op + Vector4d(0, t1->r, 0, 0);
                }
            } else
            if (o.y + t1->r > t3->box.max(2)) {
                if ((o.y - t1->r < t3->box.max(2)) && (o.x + t1->r > t3->box.min(1)) && (o.x - t1->r < t3->box.max(1)) && (o.z + t1->r > t3->box.min(3)) && (o.z - t1->r < t3->box.max(3))) {
                    N = Vector3d(0, 1, 0);
                    pp = op - Vector4d(0, t1->r, 0, 0);
                }
            } else
            if (o.z - t1->r < t3->box.min(3)) {
                if ((o.z + t1->r > t3->box.min(3)) && (o.y + t1->r > t3->box.min(2)) && (o.y - t1->r < t3->box.max(2)) && (o.x + t1->r > t3->box.min(1)) && (o.x - t1->r < t3->box.max(1))) {
                    N = Vector3d(0, 0, -1);
                    pp = op + Vector4d(0, 0, t1->r, 0);
                }
            } else
            if (o.z + t1->r > t3->box.max(3)) {
                if ((o.z - t1->r < t3->box.max(3)) && (o.y + t1->r > t3->box.min(2)) && (o.y - t1->r < t3->box.max(2)) && (o.x + t1->r > t3->box.min(1)) && (o.x - t1->r < t3->box.max(1))) {
                    N = Vector3d(0, 0, 1);
                    pp = op - Vector4d(0, 0, t1->r, 0);
                }
            }
            N = cube->Transform() * N;
            N.normalize();
            pp = cube->Transform() * pp;
            if (contact) {
                Vector3d r1(pp[0] - sphere->x[0], pp[1] - sphere->x[1], pp[2] - sphere->x[2]);
                Vector3d r2(pp[0] - cube->x[0], pp[1] - cube->x[1], pp[2] - cube->x[2]);
                Vector3d v1 = sphere->v + sphere->w.crossProduct(r1);
                Vector3d v2 = cube->v + cube->w.crossProduct(r2);
                Vector3d V(v2 - v1);
                double collid = N.dotProduct(V);
                if (fabs(collid) > -CONTACT_THRESHOLD) {
                    Contact data;
                    data.a = sphere;
                    data.b = cube;
                    data.n = N;
                    data.p = Vector3d(pp[0],pp[1],pp[2]);
                    data.ra = r1;
                    data.rb = r2;
                    data.kr = (kr < rgb->kr) ? kr : rgb->kr;
                    data.mu = (kf < rgb->kf) ? kf : rgb->kf;
                    contact->push_back(data);
                }
            }
        }
        gettimeofday(&tt2, 0);
        return;
        Matrix4d tr = rgb->Inv_Transform() * Transform();
        // face-point
        for (vector<VECTOR<float,3> >::iterator it = triangles->vertices.begin();
             it != triangles->vertices.end(); ++it) {
            Vector4d p = tr * Vector4d((*it)(1),(*it)(2),(*it)(3),1);
            pair<float, VECTOR<float,3> > intersecion = rgb->implicit_object->Intersect(VECTOR<float,3>(p[0], p[1], p[2]));

/*        for (vector<TRIANGLE<float> >::iterator it = triangles->triangle_list.begin();
             it != triangles->triangle_list.end(); ++it)
            for (int ind = 0; ind < 3; ++ind) {
                VECTOR<float, 3> &a = it->a, &b = it->b, &c = it->c;
                Vector4d p1, p2;
                if (ind == 0) {
                    p1 = tr * Vector4d(a(1), a(2), a(3), 1);
                    p2 = tr * Vector4d(b(1), b(2), b(3), 1);
                } else
                    if (ind == 1) {
                        p1 = tr * Vector4d(b(1), b(2), b(3), 1);
                        p2 = tr * Vector4d(c(1), c(2), c(3), 1);
                    } else {
                        p1 = tr * Vector4d(c(1), c(2), c(3), 1);
                        p2 = tr * Vector4d(a(1), a(2), a(3), 1);
                    }
                
                float portion;
                gettimeofday(&t1, 0);
                pair<float, VECTOR<float,3> > intersecion = rgb->implicit_object->Intersect(VECTOR<float,3>(p1[0], p1[1], p1[2]), VECTOR<float,3>(p2[0], p2[1], p2[2]), portion);
                gettimeofday(&t2, 0);
                g_collisionTime += (t2.tv_usec - t1.tv_usec) * 1e-6 + (t2.tv_sec - t1.tv_sec);
*/                if (intersecion.first > 0) {
                    continue;
                }
                if (contact) {
                    Vector3d N = rgb->Transform() * Vector3d(intersecion.second(1),intersecion.second(2),intersecion.second(3));
                    N.normalize();
                    Vector4d pp = rgb->Transform() * p;
                    //Vector4d pp = rgb->Transform() * ((p2 - p1) * portion + p1);
                    Vector3d r1(pp[0] - x[0], pp[1] - x[1], pp[2] - x[2]);
                    Vector3d r2(pp[0] - rgb->x[0], pp[1] - rgb->x[1], pp[2] - rgb->x[2]);
                    Vector3d v1 = v + w.crossProduct(r1);
                    Vector3d v2 = rgb->v + rgb->w.crossProduct(r2);
                    Vector3d V(v2 - v1);
                    double collid = N.dotProduct(V);
                    if (fabs(collid) > -CONTACT_THRESHOLD) {
                        Contact data;
                        data.a = this;
                        data.b = rgb;
                        data.n = N;
                        data.p = Vector3d(pp[0],pp[1],pp[2]);
                        data.ra = r1;
                        data.rb = r2;
                        data.kr = (kr < rgb->kr) ? kr : rgb->kr;
                        data.mu = (kf < rgb->kf) ? kf : rgb->kf;
                        contact->push_back(data);
                    }
                }
            }
        
        if (system) {
            system->collide_event(this, rgb);
        }
    }
}
