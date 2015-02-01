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
#include <set>
#include "SystemPhy.h"
#include "ARRAY.h"

class Solver
{
public:
    static void EulersStep(SystemPhy& sys, double h);
    static void QPSolve(SimLib::ARRAY<2, double>& a, SimLib::ARRAY<1, double>& b, SimLib::ARRAY<1, double>& f, SimLib::ARRAY<1, double>& c);
    static void LinearSolve(SimLib::ARRAY<2, double> a, SimLib::ARRAY<1, double> b, SimLib::ARRAY<1, double>& x);
private:
    static void fdirection(int d, SimLib::ARRAY<2, double>& a, std::set<int>& C, SimLib::ARRAY<1, double>& delta_f, SimLib::ARRAY<1, double>& delta_a);
    static pair<double, int> maxstep(SimLib::ARRAY<1, double>& f, SimLib::ARRAY<1, double>& a, SimLib::ARRAY<1, double>& delta_f, SimLib::ARRAY<1, double>& delta_a, std::set<int>& C, std::set<int>& NC, int d);
};
#endif /* defined(__simulation__Solver__) */
