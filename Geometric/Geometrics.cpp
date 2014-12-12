//
//  Geometrics.cpp
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Geometrics.h"
#include "Rigid_Geometry.h"
#include "ARRAY.h"
#include "Solver.h"

using namespace SimLib;

Geometrics::Geometrics()
{}

Geometrics::Geometrics(int n)
{
    vp.resize(n);
}

Geometrics::~Geometrics()
{
    clear();
}

void Geometrics::Display()
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
    {
        (*it)->Display();
    }
}

int Geometrics::size() const
{
    return (int)vp.size();
}

void Geometrics::addElement(Geometric* object)
{
    object->system = system;
    vp.push_back(object);
}

void Geometrics::ExcertForceField(Vector3d (*forcefunc)(Geometric*))
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        (*it)->ExcertForceField(forcefunc);
}

void Geometrics::clearForce()
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it) {
        (*it)->clearForce();
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            rgd->M = Vector3d();
        }
    }
}

Geometric* Geometrics::mouseSelect(double mouseX, double mouseY)
{
    double zbuffer = -1e30;
    Geometric* res = 0;
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
    {
        double dis = (*it)->getMouseDepth(mouseX, mouseY);
        if (dis > zbuffer && dis < EPSILON)
        {
            zbuffer = dis;
            res = (*it);
        }
    }
    if (res)
        res->setSelected(true);
    return res;
}

void Geometrics::collid_detection(Bounds& b)
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        b.collid_detection((*it));
}

void Geometrics::collid_detection(Geometrics& g) {
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it) {
        g.collid_detection((*it));
    }
    clearRemoveList();
}

void Geometrics::clearRemoveList() {
    for (set<int>::reverse_iterator it = removeList.rbegin(); it != removeList.rend(); ++it) {
        delete vp[*it];
        vp.erase(vp.begin() + (*it));
    }
    removeList.clear();
}

void Geometrics::collid_detection(Geometric* g) {
    for (int i = 0; i < vp.size(); ++i) {
        if (vp[i] != g) {
            g->collid_detection(vp[i]);
        }
    }
}

void Geometrics::contact_detection(Bounds& b)
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        b.contact_detection((*it));
}

void Geometrics::contact_detection(Geometrics& g) {
    contacts.clear();
    contacts.reserve(100);
    for (vector<Geometric*>::iterator it1 = vp.begin();
         it1 != vp.end(); ++it1) {
        Rigid_Geometry* rgd1 = dynamic_cast<Rigid_Geometry*>(*it1);
        if (rgd1) {
            for (vector<Geometric*>::iterator it2 = vp.begin();
                 it2 != vp.end(); ++it2) {
                Rigid_Geometry* rgd2 = dynamic_cast<Rigid_Geometry*>(*it2);
                if (rgd2 && rgd2 != rgd1) {
                    rgd1->collid_detection(rgd2, &contacts);
                }
            }
        }
    }
    ARRAY<1,double> b((int)contacts.size());
    for (int i = 0; i < contacts.size(); ++i) {
        Contact& c = contacts[i];
        Rigid_Geometry *A = c.a, *B = c.b;
        Vector3d &n = c.n, &ra = c.ra, &rb = c.rb;
        Vector3d &f_a = A->f, &f_b = B->f, &t_a = A->M, &t_b = B->M;
        double m1 = A->nailed ? 0 : 1 / A->mass;
        double m2 = B->nailed ? 0 : 1 / B->mass;
        Matrix3d r1 = A->rotation.rotMatrix();
        Matrix3d r2 = B->rotation.rotMatrix();
        Matrix3d J1 = A->nailed ? Matrix3d::createScale(0, 0, 0) : r1 * A->J * r1.transpose();
        Matrix3d J2 = B->nailed ? Matrix3d::createScale(0, 0, 0) : r2 * B->J * r2.transpose();
        Vector3d a_ext_part = f_a * m1 + ((J1 * t_a).crossProduct(ra));
        Vector3d b_ext_part = f_b * m2 + ((J2 * t_b).crossProduct(rb));
        Vector3d a_vel_part = A->w.crossProduct(A->w.crossProduct(ra)) + (J1 * (r1 * A->J0 * r1.transpose() * A->w)).crossProduct(ra);
        Vector3d b_vel_part = B->w.crossProduct(B->w.crossProduct(rb)) + (J2 * (r2 * B->J0 * r2.transpose() * B->w)).crossProduct(rb);
        double k1 = n.dotProduct(((a_ext_part + a_vel_part) - (b_ext_part + b_vel_part)));
        double k2 = 2 * B->w.crossProduct(n).dotProduct(A->v + A->w.crossProduct(ra) - B->v - B->w.crossProduct(rb));
        b(i+1) = k1 + k2;
    }
    ARRAY<2, double> a((int)contacts.size(), (int)contacts.size());
    for (int i = 0; i < contacts.size(); ++i) {
        for (int j = 0; j < contacts.size(); ++j) {
            Contact &ci = contacts[i];
            Contact &cj = contacts[j];
            if (ci.a != cj.a && ci.b != cj.b && ci.a != cj.b && ci.b != cj.a) {
                a(i+1,j+1) = 0;
                continue;
            }
            Rigid_Geometry *A = ci.a, *B = ci.b;
            Vector3d &ni = ci.n, &nj = cj.n, &pi = ci.p, &pj = cj.p, &ra = ci.ra, &rb = ci.rb;
            ci.ra = pi - ci.a->x;
            ci.rb = pi - ci.b->x;
            cj.ra = pj - cj.a->x;
            cj.rb = pj - cj.b->x;
            Vector3d force_a, force_b, torque_a, torque_b;
            if (cj.a == ci.a) {
                force_a = nj;
                torque_a = cj.ra.crossProduct(nj);
            } else {
                if (cj.b == ci.a) {
                    force_a = -nj;
                    torque_a = cj.rb.crossProduct(-nj);
                }
            }
            if (cj.a == ci.b) {
                force_b = nj;
                torque_b = cj.ra.crossProduct(nj);
            } else {
                if (cj.b == ci.b) {
                    force_b = -nj;
                    torque_b = cj.rb.crossProduct(-nj);
                }
            }
            
            double m1 = A->nailed ? 0 : 1 / A->mass;
            double m2 = B->nailed ? 0 : 1 / B->mass;
            Matrix3d r1 = A->rotation.rotMatrix();
            Matrix3d r2 = B->rotation.rotMatrix();
            Matrix3d J1 = A->nailed ? Matrix3d::createScale(0, 0, 0) : r1 * A->J * r1.transpose();
            Matrix3d J2 = B->nailed ? Matrix3d::createScale(0, 0, 0) : r2 * B->J * r2.transpose();
            Vector3d a_linear = force_a * m1, a_angular = (J1 * torque_a).crossProduct(ra);
            Vector3d b_linear = force_b * m2, b_angular = (J2 * torque_b).crossProduct(rb);
            a(i+1,j+1) = ni.dotProduct((a_linear + a_angular) - (b_linear + b_angular));
        }
    }
    ARRAY<1,double> f((int)contacts.size());
    Solver::QPSolve(a, b, f);
    for (int i = 0; i < contacts.size(); ++i) {
        Contact &ci = contacts[i];
        Vector3d force = ci.n * f(i+1);
        ci.a->ExcertForce(force);
        ci.b->ExcertForce(-force);
        ci.a->ExcertMoment(force, ci.p);
        ci.b->ExcertMoment(-force, ci.p);
    }
    clearRemoveList();
}

