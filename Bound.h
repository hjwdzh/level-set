//
//  Bound.h
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Bound__
#define __simulation__Bound__

#include <iostream>
#include "Geometric.h"


#define CONTACT_THRESHOLD 0.2

class Bound
{
public:
    Bound();
    virtual ~Bound() = 0;
    virtual void setKr(double _kr);
    virtual void setKf(double _kf);
    virtual bool collid_detection(Geometric* object) = 0;
    virtual bool contact_detection(Geometric* object) = 0;
    virtual void Display() = 0;
protected:
    double kr;
    double kf;
};

#endif /* defined(__simulation__Bound__) */
