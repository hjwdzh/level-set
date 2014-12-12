//
//  Springs.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Springs__
#define __simulation__Springs__

#include "Spring.h"
#include <vector>
using namespace std;

class Springs
{
public:
    Springs();
    Springs(int n);
    ~Springs();
    
    void Display();
    
    void ForceApply(double ks, double kd);
    void addElement(Geometric* p1, Geometric* p2);
    int size() const;
    void clear();
    
    vector<Spring> vs;
};

#endif /* defined(__simulation__Springs__) */
