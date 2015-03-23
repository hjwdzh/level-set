//
//  SystemPhy.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__SystemPhy__
#define __simulation__SystemPhy__

#include <iostream>
#include <vector>
#include "Geometric.h"
using namespace std;

class SystemPhy
{
public:
    enum SOLVER_TYPE {NRBS = 0, LCP = 1};
    SystemPhy();
    virtual ~SystemPhy();
    
    virtual Geometric* mouseSelect(double mouseX, double mouseY) = 0;
    virtual void Initialize() = 0;
    virtual int getDim() = 0;
    virtual double* getState() = 0;
    virtual void setState(double* state, double t) = 0;
    virtual double getTime() = 0;
    virtual void collide_event(Geometric*,Geometric*) = 0;
    virtual double* DerivEval(double* state, double t) = 0;
    virtual double* DerivVelEval(double* state, double t) {return 0;}
    virtual double* DerivPosEval(double* state, double t) {return 0;}
    
    virtual void preStabilization(double h) {}
    virtual void collide_detection() {}
    virtual void contact_handling() {}
    virtual void updateForce() {}
    virtual void postStabilization() {}
    
    virtual double* getVelState() { return 0; }
    virtual double* getPosState() { return 0; }
    virtual void setVelState(double* state, double t) {}
    virtual void setPosState(double* state, double t) {}
    
    void setSolver(SOLVER_TYPE s) { solver = s; }
    SOLVER_TYPE solver;
};

#endif /* defined(__simulation__SystemPhy__) */
