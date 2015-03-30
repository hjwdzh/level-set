//
//  RotJoint.h
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#ifndef __levelset__RotJoint__
#define __levelset__RotJoint__

#include <iostream>
#include "Joint.h"

class RotJoint : public Joint {
public:
    RotJoint();
    RotJoint(const Vector3d& p1, const Vector3d& p2, const Vector3d& _tAxis, double min_angle, double max_angle);
    virtual bool violated();
    virtual void preStabilization(double h);
    virtual bool postStabilization();
    virtual void initialize();
    double getAngle();
    double min_angle, max_angle, k;
    Vector3d tAxis, wp, wc;
    double angle;
};

#endif /* defined(__levelset__RotJoint__) */
