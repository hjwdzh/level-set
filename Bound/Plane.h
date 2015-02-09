//
//  Plane.h
//  simulation
//
//  Created by skyer on 14-4-14.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Plane__
#define __simulation__Plane__

#include <iostream>

#include "Bound.h"
#include "Geometric.h"

class Plane : public Bound
{
public:
    Plane();
    Plane(const Vector3d& P, const Vector3d& N);
    Plane(const Vector3d& _P, const Vector3d& _N, double _kr, double _kf);
    bool collide_detection(Geometric* object);
    bool contact_detection(Geometric* object);
    void Display();
private:
    Vector3d P, N;
};

#endif /* defined(__simulation__Plane__) */
