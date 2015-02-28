//
//  PtJoint.h
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#ifndef __levelset__PtJoint__
#define __levelset__PtJoint__

#include <iostream>
#include "Joint.h"

class PtJoint : public Joint {
public:
    PtJoint();
    PtJoint(Vector3d p1, Vector3d p2);
    
    virtual bool violated();
    virtual void preStabilization();
    virtual bool postStabilization();
};

#endif /* defined(__levelset__PtJoint__) */