//
//  Solver.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Solver__
#define __simulation__Solver__

#include <iostream>
#include "SystemPhy.h"

class Solver
{
public:
    static void EulersStep(SystemPhy& sys, double h);
};
#endif /* defined(__simulation__Solver__) */
