//
//  Geometrics.cpp
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Geometrics.h"

Geometrics::Geometrics()
{}

Geometrics::Geometrics(int n)
{
    vp.resize(n);
}

Geometrics::~Geometrics()
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        delete (*it);
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
         it != vp.end(); ++it)
        (*it)->clearForce();
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
         it != vp.end(); ++it)
        g.collid_detection((*it));
}

void Geometrics::collid_detection(Geometric* g) {
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it) {
        if ((*it) != g) {
            g->collid_detection(*it);
        }
    }
}

void Geometrics::contact_detection(Bounds& b)
{
    for (vector<Geometric*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        b.contact_detection((*it));
}
