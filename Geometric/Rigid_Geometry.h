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
#include "IMPLICIT_OBJECT.h"
#include "LEVELSET.h"
#include "KDOP.h"

class Contact;

class Rigid_Geometry : public Geometric {
public:
    Rigid_Geometry();
    Rigid_Geometry(const char* _name, const char* filename, const Vector3d &_x, const Vector3d &_r, const Vector3d &_s,double _m, SimLib::IMPLICIT_OBJECT<float>* object = 0, bool showLevelSet = false);
    
    double getMouseDepth(double mouseX, double mouseY);
    
    void Rotate(const Vector3d& rot);
    void Translate(const Vector3d& trans);
    void Scale(const Vector3d& s);
    
    void Display();
    void ExcertForce(const Vector3d& force);
    void ExcertMoment(const Vector3d& force, const Vector3d& p);
    void ExcertMoment(const Vector3d& moment);
    void ExcertForceField(Vector3d (*forcefunc)(Geometric*));
    void collide_detection(Geometric* b, std::vector<Contact>* contact = 0);
    void updateBoundingVolume();
    void LoadTexture(const char* bmp, double _tex_scale = 1);

    Matrix4d Transform() {
        return transform;
    }
    Matrix4d Inv_Transform() {
        return inv_transform;
    }
    
    void updateTransform();
    
    Vector3d ApplyTransform(const SimLib::VECTOR<float, 3>& p);
    
    SimLib::TRIANGULATED_SURFACE<float>* triangles;
    
    Quaternion<double> rotation;
    Vector3d scale;
    SimLib::GRID<SimLib::VECTOR<float, 3> >* grid;
    SimLib::ARRAY<3, float>* phi;
    SimLib::IMPLICIT_OBJECT<float>* implicit_object;
    SimLib::VECTOR<float, 3> gravity_center;
    SimLib::BV<float>* bounding_volume;
    
    bool show_levelset;
    Vector3d w, M;
    
    Matrix4d transform, inv_transform;
    
    Matrix3<double> J, J0;
    double kf;
    
    double tex_scale;
    unsigned int texturemap;
};

#endif /* defined(__levelset__Rigid_Geometry__) */
