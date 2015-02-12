//
//  SysDynPtc.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__SysDynPtc__
#define __simulation__SysDynPtc__

#include "Springs.h"
#include "Geometrics.h"
#include "Plane.h"
#include "Contact.h"
#include "SystemPhy.h"

class SysDynPtc: public SystemPhy
{
public:
    SysDynPtc();
    ~SysDynPtc();
    enum {COLLISION_ITERATION = 1, CONTACT_ITERATION = 9};
    virtual void Initialize();
    virtual void Display();
    virtual Geometric* mouseSelect(double mouseX, double mouseY);
    virtual int getDim();
    virtual double* getState();
    virtual void setState(double* state, double t);
    virtual void collide_event(Geometric*,Geometric*) {}
    virtual double* DerivEval(double* state, double t);

    virtual void collide_detection() {
        m_objects.collide_detection(m_objects);
    }
    virtual void contact_handling() {
        m_objects.contact_detection(m_objects);
    }
    virtual void post_initialization() {
        m_objects.updateBVH();
        m_objects.clearForce();
        ForceApply();
    }

    virtual void setVelState(double* state, double t);
    virtual void setPosState(double* state, double t);
    virtual double* getVelState();
    virtual double* getPosState();
    virtual double* DerivVelEval(double* state, double t);
    virtual double* DerivPosEval(double* state, double t);

    double getTime();
    void clear();
    double ks, kd;
    Geometrics m_objects;
    
protected:
    virtual void ForceApply();
    Springs m_springs;
    Bounds m_bounds;
    double time;
};

#endif /* defined(__simulation__SysDynPtc__) */
