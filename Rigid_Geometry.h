//
//  Rigid_Geometry.h
//  levelset
//
//  Created by Jingwei Huang on 14-10-6.
//  Copyright (c) 2014年 Jingwei Huang. All rights reserved.
//

#ifndef __levelset__Rigid_Geometry__
#define __levelset__Rigid_Geometry__

#include <iostream>
#include <string>
#include "vmath.h"
#include "TRIANGULATED_SURFACE.h"
#include "Geometric.h"
#include "LEVELSET.h"

class Rigid_Geometry : public Geometric {
public:
    Rigid_Geometry();
    Rigid_Geometry(char* name, char* filename, const Vector3d &_x, double _m);
    Rigid_Geometry(char* name, char* filename, const Vector3d &_x, const Vector3d &_v, double _m);
    
    double getMouseDepth(double mouseX, double mouseY);
    
    void Display();
    void ExcertForce(const Vector3d& force);
    void ExcertForceField(Vector3d (*forcefunc)(Geometric*));
    void collid_detection(Geometric* b);
    Matrix4d Transform() const;
    Matrix4d Inv_Transform() const;
    Vector3d ApplyTransform(SimLib::VECTOR<float, 3>& p);
    
    SimLib::TRIANGULATED_SURFACE<float> triangles;
    
    Quaternion<double> rotation;
    SimLib::GRID<SimLib::VECTOR<float, 3> > grid;
    SimLib::ARRAY<3, float> phi;
    SimLib::LEVELSET<SimLib::GRID<SimLib::VECTOR<float, 3> > > implicit_object;
    SimLib::VECTOR<float, 3> gravity_center;
    
    std::string name;
};

#endif /* defined(__levelset__Rigid_Geometry__) */
