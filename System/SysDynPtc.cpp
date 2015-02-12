//
//  SysDynPtc.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "SysDynPtc.h"
#include "ForceField.h"
#include "main.h"
#include <iostream>
#include "Rigid_Geometry.h"
#include "IMPLICIT_CUBE.h"

using namespace SimLib;

extern string res_path;
typedef VECTOR<float,3> TV;

SysDynPtc::SysDynPtc()
{
    m_objects.system = this;
}

SysDynPtc::~SysDynPtc()
{}

Geometric* SysDynPtc::mouseSelect(double mouseX, double mouseY)
{
    return m_objects.mouseSelect(mouseX, mouseY);
}

extern double g_top, g_right;
void SysDynPtc::Initialize()
{
    ks = 100; kd = 10;
    Rigid_Geometry* road1 = new Rigid_Geometry("road", (res_path + "/models/cube.obj").c_str(),Vector3d(0,-1,0),Vector3d(0,180,0),Vector3d(100,1,100),1,new IMPLICIT_CUBE<float>(RANGE<VECTOR<float,3> >(VECTOR<float,3> (-1,-1,-1),VECTOR<float,3> (1,1,1))));
    road1->setNailed();
    road1->setKr(0.1);
    road1->LoadTexture((res_path + "/texture/wood.bmp").c_str(), 0.6);   
    Rigid_Geometry* cube = new Rigid_Geometry("cube", (res_path + "/models/cube.obj").c_str(),Vector3d(0,10,0),Vector3d(0,0,0),Vector3d(1,1,1),1,new IMPLICIT_CUBE<float>(RANGE<VECTOR<float,3> >(VECTOR<float,3> (-1,-1,-1),VECTOR<float,3> (1,1,1))));
    cube->setKr(0.1);
    cube->LoadTexture((res_path + "/texture/marble.bmp").c_str(), 0.6);
    m_objects.addElement(road1);
    m_objects.addElement(cube);
}

void SysDynPtc::clear() {
    m_objects.clear();
}

void SysDynPtc::Display()
{
    m_objects.Display();
    m_springs.Display();
    m_bounds.Display();
}

int SysDynPtc::getDim()
{
    return 13 * m_objects.size();
}

double* SysDynPtc::getVelState() {
    double *state = new double[6 * m_objects.size()];
    double *t = state;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it) {
        *t++ = (*it)->v[0];
        *t++ = (*it)->v[1];
        *t++ = (*it)->v[2];
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            *t++ = rgd->w[0];
            *t++ = rgd->w[1];
            *t++ = rgd->w[2];
        } else {
            t += 3;
        }
    }
    return state;
}

double* SysDynPtc::getPosState() {
    double *state = new double[7 * m_objects.size()];
    double *t = state;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it) {
        *t++ = (*it)->x[0];
        *t++ = (*it)->x[1];
        *t++ = (*it)->x[2];
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            *t++ = rgd->rotation.v[0];
            *t++ = rgd->rotation.v[1];
            *t++ = rgd->rotation.v[2];
            *t++ = rgd->rotation.w;
        } else {
            t += 4;
        }
    }
    return state;
}

double* SysDynPtc::getState()
{
    double *state = new double[13 * m_objects.size()];
    double* t = state;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it)
    {
        *t++ = (*it)->x[0];
        *t++ = (*it)->x[1];
        *t++ = (*it)->x[2];
        *t++ = (*it)->v[0];
        *t++ = (*it)->v[1];
        *t++ = (*it)->v[2];
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            *t++ = rgd->rotation.v[0];
            *t++ = rgd->rotation.v[1];
            *t++ = rgd->rotation.v[2];
            *t++ = rgd->rotation.w;
            *t++ = rgd->w[0];
            *t++ = rgd->w[1];
            *t++ = rgd->w[2];
        } else {
            t += 7;
        }
    }
    return state;
}

void SysDynPtc::setVelState(double* state, double t) {
    double* pt = state;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it) {
        (*it)->v[0] = *pt++;
        (*it)->v[1] = *pt++;
        (*it)->v[2] = *pt++;
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            rgd->w[0] = *pt++;
            rgd->w[1] = *pt++;
            rgd->w[2] = *pt++;
        }
    }
    time = t;
}

void SysDynPtc::setPosState(double* state, double t) {
    double* pt = state;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it) {
        (*it)->x[0] = *pt++;
        (*it)->x[1] = *pt++;
        (*it)->x[2] = *pt++;
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            rgd->rotation.v[0] = *pt++;
            rgd->rotation.v[1] = *pt++;
            rgd->rotation.v[2] = *pt++;
            rgd->rotation.w = *pt++;
            rgd->rotation.normalize();
        }
    }
    time = t;
}

void SysDynPtc::setState(double* state, double t)
{
    double* pt = state;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it)
    {
        (*it)->x[0] = *pt++;
        (*it)->x[1] = *pt++;
        (*it)->x[2] = *pt++;
        (*it)->v[0] = *pt++;
        (*it)->v[1] = *pt++;
        (*it)->v[2] = *pt++;
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            rgd->rotation.v[0] = *pt++;
            rgd->rotation.v[1] = *pt++;
            rgd->rotation.v[2] = *pt++;
            rgd->rotation.w = *pt++;
            rgd->rotation.normalize();
            rgd->w[0] = *pt++;
            rgd->w[1] = *pt++;
            rgd->w[2] = *pt++;
        }
/*        for (int i = 0; i < 3; ++i) {
            if (abs(rgd->v[i]) < 3e-2) {
                rgd->v[i] = 0;
            }
            if (abs(rgd->w[i]) < 3e-2) {
                rgd->w[i] = 0;
            }
        }
*/    }
    
    delete[] state;
    time += t;
    
}

double* SysDynPtc::DerivVelEval(double* state, double t)
{
    double *delta = new double[6 * m_objects.size()];
    double *st = delta;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it)
    {
        *st++ = (*it)->f[0] / (*it)->mass;
        *st++ = (*it)->f[1] / (*it)->mass;
        *st++ = (*it)->f[2] / (*it)->mass;
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            Matrix3d rotation = rgd->rotation.rotMatrix();
            Vector3d dw = (rotation * rgd->J * rotation.transpose())* rgd->M;
            *st++ = dw[0];
            *st++ = dw[1];
            *st++ = dw[2];
        } else {
            st += 3;
        }
    }
    return delta;
}

double* SysDynPtc::DerivPosEval(double* state, double t)
{
    double *delta = new double[13 * m_objects.size()];
    double *st = delta;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it)
    {
        *st++ = (*it)->v[0];
        *st++ = (*it)->v[1];
        *st++ = (*it)->v[2];
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            *st++ = rgd->w[0];
            *st++ = rgd->w[1];
            *st++ = rgd->w[2];
            *st++ = 0;
        } else {
            st += 4;
        }
    }
    return delta;
}

double* SysDynPtc::DerivEval(double* state, double t)
{
    double *delta = new double[13 * m_objects.size()];
    double *st = delta;
    for (vector<Geometric*>::iterator it = m_objects.vp.begin();
         it != m_objects.vp.end(); ++it)
    {
        *st++ = (*it)->v[0];
        *st++ = (*it)->v[1];
        *st++ = (*it)->v[2];
        *st++ = (*it)->f[0] / (*it)->mass;
        *st++ = (*it)->f[1] / (*it)->mass;
        *st++ = (*it)->f[2] / (*it)->mass;
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(*it);
        if (rgd) {
            *st++ = rgd->w[0];
            *st++ = rgd->w[1];
            *st++ = rgd->w[2];
            *st++ = 0;
            Matrix3d rotation = rgd->rotation.rotMatrix();
            Vector3d dw = (rotation * rgd->J * rotation.transpose())* rgd->M;
            *st++ = dw[0];
            *st++ = dw[1];
            *st++ = dw[2];
        } else {
            st += 7;
        }
    }
    return delta;
}

void SysDynPtc::ForceApply()
{
    m_objects.ExcertForceField(ForceField::gravity);
    m_objects.ExcertForceField(ForceField::viscous);
    for (int i = 0; i < m_objects.size(); ++i) {
        Rigid_Geometry* rgd = dynamic_cast<Rigid_Geometry*>(m_objects.vp[i]);
        if (rgd) {
            rgd->ExcertMoment(-rgd->w * ForceField::k_moment_drag);
        }
        
    }
    m_springs.ForceApply(ks, kd);
}

double SysDynPtc::getTime()
{
    return time;
}
