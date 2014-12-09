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
    Rigid_Geometry* rgd2 = new Rigid_Geometry("base", "models/cube.obj",Vector3d(-2,0,-3),1);
    rgd2->Scale(Vector3d(20,1,20));
    rgd2->Translate(Vector3d(0,0,0));
    rgd2->setNailed();
    typedef SimLib::VECTOR<float,3> TV;
    Rigid_Geometry* rgd1 = new Rigid_Geometry("obj2", "models/cube.obj",Vector3d(-2,10,-3),1);
    Rigid_Geometry* rgd3 = new Rigid_Geometry("obj1", "models/cube.obj",Vector3d(-2,5,-3),1);
    Rigid_Geometry* rgd4 = new Rigid_Geometry("obj3", "models/cube.obj",Vector3d(-2,15,-3),1);
    Rigid_Geometry* rgd5 = new Rigid_Geometry("obj4", "models/cube.obj",Vector3d(-2,20,-3),1);
    Rigid_Geometry* rgd6 = new Rigid_Geometry("obj5", "models/cube.obj",Vector3d(-2,25,-3),1);
//    rgd->Translate(Vector3d(0, 5, 0));
    m_objects.addElement(rgd1);
    m_objects.addElement(rgd2);
    m_objects.addElement(rgd6);
    m_objects.addElement(rgd3);
    m_objects.addElement(rgd4);
    m_objects.addElement(rgd5);
//    m_objects.addElement(rgd3);
//    m_bounds.addElement(new Plane(Vector3d(0, 0, 0), Vector3d(0, 1, 0), 0.5, 0));
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
