//
//  Particle.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Particles__
#define __simulation__Particles__
#include <iostream>
#include <vector>
using namespace std;

#include "Geometric.h"
using namespace std;

class Particle : public Geometric
{
public:
    Particle();
    Particle(const Vector3d &_x, double _m);
    Particle(const Vector3d &_x, const Vector3d &_v, double _m);
    
    double getMouseDepth(double mouseX, double mouseY);
    
    void Display();    
    void ExcertForce(const Vector3d& force);
    void ExcertForceField(Vector3d (*forcefunc)(Geometric*));
     
};


#endif /* defined(__simulation__Particles__) */
