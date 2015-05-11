//
//  ForceField.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__ForceField__
#define __simulation__ForceField__

#include <iostream>

#include "Particle.h"

class ForceField
{
public:
    static Vector3d gravity(Geometric* p);
    static Vector3d viscous(Geometric* p);
    static Vector3d viscousMoment(Geometric* p);
    static double k_drag;
    static double k_moment_drag;
};

#endif /* defined(__simulation__ForceField__) */
