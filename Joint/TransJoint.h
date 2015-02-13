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
    virtual void preStabilization();
    virtual void postStabilization();
    
    double min_dis, max_dis, k;
    Vector3d tPos;
    Quaternion<double> tAngle;
};

#endif /* defined(__levelset__TransJoint__) */
