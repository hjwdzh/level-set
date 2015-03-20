//
//  TransJoint.h
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#ifndef __levelset__TransJoint__
#define __levelset__TransJoint__

#include <iostream>

#include "Joint.h"

class TransJoint : public Joint {
public:
    virtual bool violated();
    virtual void preStabilization(double h);
    virtual bool postStabilization();
    virtual double getDistance();
    virtual void initialize();
    
    double min_dis, max_dis, k;
    
    Vector4d pX, cX;
    Vector3d tAxis, vp, vc;
    double dis;
};

#endif /* defined(__levelset__TransJoint__) */
