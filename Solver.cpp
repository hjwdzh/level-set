//
//  Solver.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Solver.h"

void Solver::EulersStep(SystemPhy &sys, double h)
{
    double t = sys.getTime();
    int n = sys.getDim();
    double* x = sys.getState();
    double* deltaX = sys.DerivEval(x, t);
    for (int i = 0; i < n; ++i)
    {
        x[i] += h * deltaX[i];
    }
    sys.setState(x, t + h);
}
