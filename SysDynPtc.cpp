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
#include "Rigid_Geometry.h"

SysDynPtc::SysDynPtc()
{}

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
//    m_objects.addElement(new Rigid_Geometry("obj2", "/Users/jingweihuang/Desktop/projects/levelset/models/monkey.obj",Vector3d(-2.5,10.5,-3),Vector3d(0,0,0),1));
    Rigid_Geometry* rgd2 = new Rigid_Geometry("base", "/Users/jingweihuang/Desktop/projects/levelset/models/cube.obj",Vector3d(0,-1,0),Vector3d(0,0,0),Vector3d(1,1,1),1);
    Rigid_Geometry* rgd1 = new Rigid_Geometry("base", "/Users/jingweihuang/Desktop/projects/levelset/models/sphere.obj",Vector3d(0,0,-10),Vector3d(0,0,0),Vector3d(1,1,1),10);
    rgd1->setKr(0.3);
    rgd1->v = Vector3d(0,0,8);
    rgd2->Scale(Vector3d(200,1,200));
    rgd2->Translate(Vector3d(0,0,0));
    rgd2->setNailed();
    typedef SimLib::VECTOR<float,3> TV;
    Rigid_Geometry* bowling[10];
    int t = 0;
    double scale = 5;
    for (int i = 1; i <= 4; ++i) {
        for (int j = 1; j <= 5 - i; ++j) {
            double h = -scale * cos(3.14159265 / 12) * i / 4;
            double x = 0.5 * scale * (j - 1 - (4 - i) * 0.5);
            bowling[t++] = new Rigid_Geometry("bowling", "/Users/jingweihuang/Desktop/projects/levelset/models/bowling.obj",Vector3d(x,0,h),Vector3d(0,0,0),Vector3d(5,5,5),1);
        }
    }
    for (int i = 0; i < t; ++i) {
        m_objects.addElement(bowling[i]);
    }

    m_objects.addElement(rgd1);
    m_objects.addElement(rgd2);
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
        } else {
            t += 7;
        }
    }
    
    delete[] state;
    time = t;
    for (int i = 0; i < COLLISION_ITERATION - 1; ++i) {
        m_objects.collid_detection(m_objects);
        m_objects.collid_detection(m_bounds);
    }
    m_objects.clearForce();
    ForceApply();
    m_objects.contact_detection(m_bounds);
    m_objects.contact_detection(m_objects);
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
