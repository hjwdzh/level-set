//
//  Spring.h
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Spring__
#define __simulation__Spring__

#include <iostream>
#include "Geometric.h"

class Spring
{
public:
    Geometric *p1, *p2;
    double r;
    void Display();
    void ForceApply(double ks, double kd);
};

#endif /* defined(__simulation__Spring__) */
