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
    TransJoint();
    TransJoint(const Vector3d& p1, const Vector3d& p2, const Vector3d& _tAxis, double min_dis, double max_dis);
    virtual void ExcertForce();
    virtual bool violated();
    virtual void preStabilization(double h);
    virtual bool postStabilization();
    virtual Vector4d getDistance();
    virtual void initialize();
    
    double min_dis, max_dis, k;
    
    Vector4d pX, cX;
    Vector3d tAxis, vp, vc, dp;
    Vector4d dis;
};

#endif /* defined(__levelset__TransJoint__) */
