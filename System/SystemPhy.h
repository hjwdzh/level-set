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
    SystemPhy();
    virtual ~SystemPhy();
    
    virtual Geometric* mouseSelect(double mouseX, double mouseY) = 0;
    virtual void Initialize() = 0;
    virtual int getDim() = 0;
    virtual double* getState() = 0;
    virtual void setState(double* state, double t) = 0;
    virtual double getTime() = 0;
    virtual void collid_event(Geometric*,Geometric*) = 0;
    virtual double* DerivEval(double* state, double t) = 0;
};

#endif /* defined(__simulation__SystemPhy__) */
