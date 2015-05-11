//
//  Springs.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//
#include "main.h"
#include "Springs.h"

Springs::Springs()
{}

Springs::Springs(int n)
{
    vs.resize(n);
}

Springs::~Springs()
{}

int Springs::size() const
{
    return (int)vs.size();
}

void Springs::Display()
{
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    for (vector<Spring>::iterator it = vs.begin();
         it != vs.end(); ++it)
        it->Display();
    glEnd();
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
}

void Springs::ForceApply(double ks, double kd)
{
    for (vector<Spring>::iterator it = vs.begin();
         it != vs.end(); ++it)
        it->ForceApply(ks, kd);
}

void Springs::addElement(Geometric *p1, Geometric *p2)
{
    Spring s;
    s.p1 = p1;
    s.p2 = p2;
    s.r = (p1->x - p2->x).length();
    vs.push_back(s);
}

void Springs::clear()
{
    vs.clear();
}

