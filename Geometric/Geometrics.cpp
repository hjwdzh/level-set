//
//  Geometrics.cpp
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Geometrics.h"
#include "Rigid_Geometry.h"
#include "BVH.h"
#include "ARRAY.h"
#include "Solver.h"
#include "main.h"
#include "ContactGraph.h"
#include <sys/time.h>
extern double g_simTime;
using namespace SimLib;

Geometrics::Geometrics()
: bvh(new BVH<3, float>())
{}

Geometrics::Geometrics(int n)
: bvh(new BVH<3, float>())
{
    vp.resize(n);
}

Geometrics::~Geometrics()
{
    clear();
}

void Geometrics::Display()
{
    for (vector<Geometric*>::reverse_iterator it = vp.rbegin();
         it != vp.rend(); ++it)
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

void Geometrics::collide_detection(Bounds& b)
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        b.collide_detection((*it));
}

void Geometrics::collide_detection(Geometrics& g) {
    contacts.clear();
    contacts.reserve(100);
    bvh->collide_detection(&contacts);
    bool has_collision = true;
    int iteration = 0;
    while (has_collision) {
        has_collision = false;
        iteration++;
        if (iteration > 2)
            break;
        for (int i = 0; i < contacts.size(); ++i) {
            if (contacts[i].collide_handling()) {
                has_collision = true;
            }
        }
    }
}

void Geometrics::clearRemoveList() {
    for (set<int>::reverse_iterator it = removeList.rbegin(); it != removeList.rend(); ++it) {
        delete vp[*it];
        vp.erase(vp.begin() + (*it));
    }
    removeList.clear();
}

void Geometrics::collide_detection(Geometric* g) {
}

void Geometrics::contact_detection(Bounds& b)
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        b.contact_detection((*it));
}

void Geometrics::contact_detection(Geometrics& g) {
    if (system->solver == SystemPhy::NRBS) {
/*        for (double l = -0.6; l < 1e-3; l += 0.3) {
            for (int i = 0; i < contacts.size(); ++i) {
                contacts[i].collide_handling(l);
            }
        }
        bool has_collision = true;
        int iteration = 0;
        while (has_collision) {
            iteration++;
            if (iteration > 1)
                break;
            has_collision = false;
            for (int i = 0; i < contacts.size(); ++i) {
                if (contacts[i].collide_handling(0)) {
                    has_collision = true;
                }
            }
        }
*/        ContactGraph cg;
/*        for (double l = -0.6; l < 1e-3; l += 0.3) {
            for (int i = 0; i < contacts.size(); ++i) {
                contacts[i].collide_handling(l);
            }
        }
*/        for (int i = 0; i < contacts.size(); ++i) {
            if (contacts[i].collide_handling(0, false)) {
                cg.Connect(contacts[i]);
            }
        }
        cg.solveContacts();
    } else {
        Contact::contact_handling(contacts);
    }
    clearRemoveList();
}

void Geometrics::updateBVHandTransform() {
    std::vector<BV<float>*> bvs;
    bvs.reserve(vp.size());
    for (int i = 0; i < vp.size(); ++i) {
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(vp[i]);
        if (rgd) {
            rgd->updateTransform();
            BV<float>* bv = rgd->bounding_volume->transform(rgd->transform);
            bv->rgd = rgd;
            bvs.push_back(bv);
        }
    }
    bvh->updateBVH(bvs, 1, -1, -1);
}