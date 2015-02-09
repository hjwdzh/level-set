//
//  Plane.cpp
//  simulation
//
//  Created by skyer on 14-4-14.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Plane.h"
#include "Particle.h"
#include "main.h"
#include "Rigid_Geometry.h"
#include <vector>
using namespace std;
using namespace SimLib;

Plane::Plane()
    : N(0, 1, 0)
{}

Plane::Plane(const Vector3d& _P, const Vector3d& _N)
    : P(_P), N(_N)
{}

Plane::Plane(const Vector3d& _P, const Vector3d& _N, double _kr, double _kf)
: P(_P), N(_N)
{
    kr = _kr;
    kf = _kf;
}

bool Plane::collide_detection(Geometric* object)
{
    Particle* ptc = dynamic_cast<Particle*>(object);
    if (ptc)
    {
        double dis = (ptc->x - P).dotProduct(N);
        double collid = N.dotProduct(ptc->v) / ptc->v.length();
        if (dis < EPSILON && collid < -EPSILON)
        {
            Vector3d vn = N * N.dotProduct(ptc->v);
            Vector3d vt = ptc->v - vn;
            ptc->v = vt - vn * (kr * ptc->kr);
            ptc->x = ptc->x - N * dis;
        }
    }
    Rigid_Geometry* rgb = dynamic_cast<Rigid_Geometry*>(object);
    if (rgb)
    {
        double dis = 1e30;
        Vector4d cp;
        for (vector<VECTOR<float, 3> >::iterator it = rgb->triangles->vertices.begin();
             it != rgb->triangles->vertices.end(); ++it) {
            double t = (rgb->ApplyTransform(*it) - P).dotProduct(N);
            if (t < dis) {
                dis = t;
                cp = Vector4d((*it)(1),(*it)(2),(*it)(3),1);
            }
        }
        Vector3d r1(cp[0],cp[1],cp[2]);
        Matrix3d rot1 = rgb->rotation.rotMatrix();
        Matrix3d J1 = rot1 * rgb->J * rot1.transpose();
        double m1 = 1 / rgb->mass;
        Matrix3d crossM1 = Matrix3d::createCrossProductMatrix(r1);
        Vector3d V = -(rgb->v + crossM1.transpose() * rgb->w);
        double collid = N.dotProduct(V);
        if (dis < EPSILON && collid > EPSILON)
        {
            cp = rgb->Transform() * cp;
            Matrix3d K = Matrix3d() * m1 + crossM1.transpose() * J1 * crossM1;
            double jn = (1 + min(kr, rgb->kr)) * collid / N.dotProduct(K * N);
            rgb->v += N * (jn * m1);
            rgb->w += J1 * r1.crossProduct(N) * jn;

            Vector3d Vt = V - N * V.dotProduct(N);
            if (Vt.length() < 1e-4)
                return true;
            Vector3d Vtn = Vt;
            Vtn.normalize();
            double jt = Vt.length() / Vtn.dotProduct(K * Vtn);
            if (jt > jn * max(kf, rgb->kf)) {
                jt = jn * max(kf, rgb->kf);
            }
            rgb->v += Vtn * (jt * m1);
            rgb->w += J1 * (r1.crossProduct(Vtn) * jt);
        }
    }
    return false;
}

bool Plane::contact_detection(Geometric* object)
{
    Particle* ptc = dynamic_cast<Particle*>(object);
    if (ptc)
    {
        double dis = (ptc->x - P).dotProduct(N);
        double collid = N.dotProduct(ptc->v);
        if (dis < EPSILON && abs(collid) < CONTACT_THRESHOLD)
        {
            ptc->x -= N * dis;
            ptc->v -= N * collid;
            double fn = N.dotProduct(ptc->f);
            if (fn < 0)
                object->ExcertForce(N * -fn);
        }
    }
    return false;
}

void Plane::Display()
{
    Vector3d x(0.3, 1.4, 5.3);
    Vector3d y = x.crossProduct(N) * 10000;
    x = y.crossProduct(N);
    glColor3f(0.0f, 0.5f, 0.2f);
    glBegin(GL_QUADS);
        glNormal3d(N[0], N[1], N[2]);
        glVertex3d(P[0] + x[0] + y[0], P[1] + x[1] + y[1], P[2] + x[2] + y[2]);
        glNormal3d(N[0], N[1], N[2]);
        glVertex3d(P[0] + x[0] - y[0], P[1] + x[1] - y[1], P[2] + x[2] - y[2]);
        glNormal3d(N[0], N[1], N[2]);
        glVertex3d(P[0] - x[0] - y[0], P[1] - x[1] - y[1], P[2] - x[2] - y[2]);
        glNormal3d(N[0], N[1], N[2]);
        glVertex3d(P[0] - x[0] + y[0], P[1] - x[1] + y[1], P[2] - x[2] + y[2]);
    glEnd();
}