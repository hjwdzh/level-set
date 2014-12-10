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
#include <map>
#include <vector>
#include "vmath.h"
#include "TRIANGULATED_SURFACE.h"
#include "Geometric.h"
#include "LEVELSET.h"

class Contact;

class Rigid_Geometry : public Geometric {
public:
    Rigid_Geometry();
    Rigid_Geometry(const char* name, const char* filename, const Vector3d &_x, double _m, bool showLevelSet = false);
    Rigid_Geometry(const char* name, const char* filename, const Vector3d &_x, const Vector3d &_v, double _m, bool showLevelSet = false);
    
    double getMouseDepth(double mouseX, double mouseY);
    
    void Rotate(const Vector3d& rot);
    void Translate(const Vector3d& trans);
    void Scale(const Vector3d& s);
    
    void Display();
    void ExcertForce(const Vector3d& force);
    void ExcertMoment(const Vector3d& force, const Vector3d& p);
    void ExcertMoment(const Vector3d& moment);
    void ExcertForceField(Vector3d (*forcefunc)(Geometric*));
    void collid_detection(Geometric* b, std::vector<Contact>* contact = 0);
//    void update_velocity();
    Matrix4d Transform() const;
    Matrix4d Inv_Transform() const;
    Vector3d ApplyTransform(const SimLib::VECTOR<float, 3>& p);
    
    SimLib::TRIANGULATED_SURFACE<float> triangles;
    
    Quaternion<double> rotation;
    Vector3d scale;
    SimLib::GRID<SimLib::VECTOR<float, 3> > grid;
    SimLib::ARRAY<3, float> phi;
    SimLib::LEVELSET<SimLib::GRID<SimLib::VECTOR<float, 3> > > implicit_object;
    SimLib::VECTOR<float, 3> gravity_center;
    
    std::string name;
    bool show_levelset;
    Vector3d w, M;
    
    Matrix3<double> J, J0;
    double kf;
};

class Contact {
public:
    Contact()
    : a(0), b(0) {
    }
    Rigid_Geometry *a, *b;
    Vector3d p, n, ra, rb;
};

#endif /* defined(__levelset__Rigid_Geometry__) */