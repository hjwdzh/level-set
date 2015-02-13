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
    virtual bool violated();
    virtual void preStabilization();
    virtual void postStabilization();

    double min_angle, max_angle, k;
    Vector3d tPos;
    Quaternion<double> tAngle;
};

#endif /* defined(__levelset__RotJoint__) */
