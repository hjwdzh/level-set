//
//  Spring.cpp
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//
#include "main.h"
#include "Spring.h"

void Spring::ForceApply(double ks, double kd)
{
    Vector3d dx = p1->x - p2->x;
    Vector3d dv = p1->v - p2->v;
    double len = dx.length();
    Vector3d f2 = dx * ((ks * (len - r) + (dv.dotProduct(dx)) * (kd / len)) / len);
    Vector3d f1 = -f2;
    if (f1[0] != 0 || f1[1] != 0 || f1[2] != 0)
    {
        int t = 0;
        t = t + 1;
    }
    p1->ExcertForce(f1);
    p2->ExcertForce(f2);
}

void Spring::Display()
{
	glVertex3d(p1->x[0], p1->x[1], p1->x[2]);
	glVertex3d(p2->x[0], p2->x[1], p2->x[2]);
}
