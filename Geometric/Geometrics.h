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
#include <set>
#include "Geometric.h"
#include "Rigid_Geometry.h"
#include "Bounds.h"
using namespace std;

class SystemPhy;
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
    std::vector<Geometric*> findByName(const char* name) {
        std::vector<Geometric*> ret;
        for (int i = 0; i < (int)vp.size(); ++i) {
            if (strcmp(vp[i]->name.c_str(),name) == 0) {
                ret.push_back(vp[i]);
            }
        }
        return ret;
    }
    void removeByName(const char* name, bool clear = false) {
        for (int i = 0; i < vp.size(); ++i) {
            if (strcmp(vp[i]->name.c_str(),name) == 0) {
                removeList.insert(i);
            }
        }
    }
    void clear() {
        for (int i = 0; i < vp.size(); ++i) {
            delete vp[i];
        }
        vp.clear();
    }
    void clearRemoveList();
    
    void updateBVH();
    vector<Geometric*> vp;
    set<int> removeList;
    vector<Contact> contacts;
    SystemPhy* system;
};


#endif /* defined(__simulation__Geometrics__) */
