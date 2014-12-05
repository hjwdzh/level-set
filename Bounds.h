//
//  Bounds.h
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Bounds__
#define __simulation__Bounds__

#include "Bound.h"
#include <iostream>
#include <vector>
using namespace std;

class Bounds
{
public:
    Bounds();
    ~Bounds();
    int size() const;
    void addElement(Bound* b);
    void Display();
    bool collid_detection(Geometric* object);
    bool contact_detection(Geometric* object);
    vector<Bound*> vp;
};

#endif /* defined(__simulation__Bounds__) */
