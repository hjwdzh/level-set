//
//  Geometric.h
//  simulation
//
//  Created by skyer on 14-4-14.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Geometric__
#define __simulation__Geometric__

#include <iostream>
#include <vector>
#include "vmath.h"

class Contact;

class Geometric
{
public:
    Geometric();
    virtual ~Geometric() = 0;
    virtual void setSelected(bool selected);
    virtual void setNailed();
    virtual void setExtForce(const Vector3d& ext);
    virtual void addExtForce(const Vector3d& ext);
    virtual void clearForce();
    virtual void setUserForce(const Vector3d& v);
    virtual void setKr(double _kr);
    virtual double getMouseDepth(double mouseX, double mouseY) = 0;
    virtual void Display() = 0;
    virtual void ExcertForce(const Vector3d& force) = 0;
    virtual void ExcertForceField(Vector3d (*forcefunc)(Geometric*)) = 0;
    virtual void collid_detection(Geometric* g, std::vector<Contact>* contact = 0);
    virtual void contact_detection(Geometric* g);
    double mass, kr;
    bool selected, nailed;
    Vector3d x, v, f, extForce, userForce;
    
};


#endif /* defined(__simulation__Geometric__) */
