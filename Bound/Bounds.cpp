//
//  Bounds.cpp
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Bounds.h"

Bounds::Bounds()
{}

Bounds::~Bounds()
{
    for (vector<Bound*>::iterator it = vp.begin();
         it != vp.end(); ++it)
    {
        delete (*it);
    }
}

void Bounds::addElement(Bound* b)
{
    vp.push_back(b);
}

int Bounds::size() const
{
    return (int)vp.size();
}

bool Bounds::collid_detection(Geometric* object)
{
    bool t = false;
    for (vector<Bound*>::iterator it = vp.begin();
         it != vp.end(); ++it)
    {
        t |= (*it)->collid_detection(object);
    }
    return t;
}

bool Bounds::contact_detection(Geometric* object)
{
    bool t = false;
    for (vector<Bound*>::iterator it = vp.begin();
         it != vp.end(); ++it)
    {
        t |= (*it)->contact_detection(object);
    }
    return t;
}

void Bounds::Display()
{
    for (vector<Bound*>::iterator it = vp.begin();
         it != vp.end(); ++it)
        (*it)->Display();
}