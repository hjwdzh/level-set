//
//  Joint.h
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#ifndef __levelset__Joint__
#define __levelset__Joint__

#include <iostream>
#include "vmath.h"
#include "Rigid_Geometry.h"

class Joint {
public:
    virtual bool violated() = 0;
    virtual void preStabilization() = 0;
    virtual void postStabilization() = 0;
    
    Rigid_Geometry *parent, *child;
    Vector3d pPos, cPos;
    Quaternion<double> pAngle, cAngle;
};

#endif /* defined(__levelset__Joint__) */
