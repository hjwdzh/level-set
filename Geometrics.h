//
//  Geometrics.h
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Geometrics__
#define __simulation__Geometrics__

#include <iostream>
#include <vector>
#include "Geometric.h"
#include "Rigid_Geometry.h"
#include "Bounds.h"
using namespace std;

class Geometrics
{
public:
    Geometrics();
    Geometrics(int n);
    ~Geometrics();
    
    void Display();
    
    Geometric* mouseSelect(double mouseX, double mouseY);
    void collid_detection(Bounds& b);
    void collid_detection(Geometrics& b);
    void collid_detection(Geometric* b);
    void contact_detection(Bounds& b);
    void contact_detection(Geometrics& b);
    void addElement(Geometric* object);
    
    void clearForce();
    void ExcertForceField(Vector3d (*forcefunc)(Geometric*));
    
    int size() const;
    void clear();
    
    vector<Geometric*> vp;
    vector<Contact> contacts;
};


#endif /* defined(__simulation__Geometrics__) */
