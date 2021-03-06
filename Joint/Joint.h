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
#include "ARRAY.h"

class Joint {
public:
    Joint();
    virtual bool violated();
    virtual void preStabilization(double h);
    virtual bool postStabilization();
    virtual void initialize();
    virtual void ExcertForce();
    Rigid_Geometry *parent, *child;
    Vector3d pPos, cPos, dp, dc;
    Quatd qt;
    double kr, kf, kh;
protected:
    virtual Vector3d f(double h, const Vector3d& j);
    virtual Quatd ft(double h, const Vector3d& jt);
    virtual void dfj(double h, const Vector3d& j, Matrix3d& r);
    virtual void dfjt(double h, const Vector3d& jt, std::pair<Vector3d,Matrix3d>& r);
    virtual Vector3d solvej(double h);
    virtual Vector3d solvejt(double h);
};

#endif /* defined(__levelset__Joint__) */
