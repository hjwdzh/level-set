//
//  ForceField.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "ForceField.h"
#include "Rigid_Geometry.h"
double ForceField::k_drag = 0;
double ForceField::k_moment_drag = 0;

Vector3d ForceField::gravity(Geometric* p)
{
    return Vector3d(0, -9.8 * p->mass, 0);
}

Vector3d ForceField::viscous(Geometric* p)
{
    return -p->v * k_drag;
}

Vector3d ForceField::viscousMoment(Geometric* p) {
    Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(p);
    if (rgd)
        return -rgd->w * k_moment_drag;
    return Vector3d();
}