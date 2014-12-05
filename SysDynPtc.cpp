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
    /* scene 3 */
    m_objects.addElement(new Rigid_Geometry("obj1", "/Users/jingweihuang/Desktop/projects/levelset/models/rsphere.obj",Vector3d(-2,1.5,-3.0),1));
    m_objects.addElement(new Rigid_Geometry("obj2", "/Users/jingweihuang/Desktop/projects/levelset/models/rsphere.obj",Vector3d(2,1.5,-4.41),Vector3d(-4,0,0),1));
/*    m_objects.addElement(new Particle(Vector3d(-0.5,5,-4), 1));
    m_objects.addElement(new Particle(Vector3d(-0.5,6,-4), 1));
    m_objects.addElement(new Particle(Vector3d(0.5,6,-4), 1));
    m_objects.addElement(new Particle(Vector3d(0.5,5,-4), 1));
    m_objects.addElement(new Particle(Vector3d(-0.5,5,-5), 1));
    m_objects.addElement(new Particle(Vector3d(-0.5,6,-5), 1));
    m_objects.addElement(new Particle(Vector3d(0.5,6,-5), 1));
    m_objects.addElement(new Particle(Vector3d(0.5,5,-5), 1));
/*    m_springs.addElement(m_objects.vp[0],m_objects.vp[1]);
    m_springs.addElement(m_objects.vp[1],m_objects.vp[2]);
    m_springs.addElement(m_objects.vp[2],m_objects.vp[3]);
    m_springs.addElement(m_objects.vp[3],m_objects.vp[0]);
    m_springs.addElement(m_objects.vp[4],m_objects.vp[5]);
    m_springs.addElement(m_objects.vp[5],m_objects.vp[6]);
//    m_springs.addElement(m_objects.vp[6],m_objects.vp[7]);
//    m_springs.addElement(m_objects.vp[7],m_objects.vp[4]);
    m_springs.addElement(m_objects.vp[0],m_objects.vp[4]);
    m_springs.addElement(m_objects.vp[1],m_objects.vp[5]);
    m_springs.addElement(m_objects.vp[2],m_objects.vp[6]);
    m_springs.addElement(m_objects.vp[3],m_objects.vp[7]);*/
    m_bounds.addElement(new Plane(Vector3d(0, 0, 0), Vector3d(0, 1, 0), 0.5));

    /* scene 2 */
/*    m_objects.addElement(Vector3d(-0.5,-0.5,-2), 1);
    m_objects.addElement(Vector3d(-0.5,0.5,-2), 1);
    m_objects.addElement(Vector3d(0.5,0.5,-2), 1);
    m_objects.addElement(Vector3d(0.5,-0.5,-2), 1);
    m_springs.addElement(m_objects.vp[0],m_objects.vp[1]);
    m_springs.addElement(m_objects.vp[1],m_objects.vp[2]);
    m_springs.addElement(m_objects.vp[2],m_objects.vp[3]);
    m_springs.addElement(m_objects.vp[3],m_objects.vp[0]);
    m_objects.vp[0].setExtForce(Vector3d(-0.3, 0.3, 0));
    m_objects.vp[1].setExtForce(Vector3d(0.3, 0.3, 0));
    m_objects.vp[2].setExtForce(Vector3d(0.3, -0.3, 0));
    m_objects.vp[3].setExtForce(Vector3d(-0.3, -0.3, 0));
*/
    /* scene 1 */
/*    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
        {
            Particle p;
            Vector3d x(0.4 * i - 0.8, 0.4 * j - 0.8, 0);
            p.x = x;
            m_objects.addElement(x, 1);
        }
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 4; ++j)
        {
            m_springs.addElement(m_objects.vp[i * 5 + j], m_objects.vp[i * 5 + j + 1]);
            m_springs.addElement(m_objects.vp[i + j * 5], m_objects.vp[i + (j + 1) * 5]);
        }
*/
}

void SysDynPtc::Display()
{
    m_objects.Display();
    m_springs.Display();
    m_bounds.Display();
}

int SysDynPtc::getDim()
{
    return 6 * m_objects.size();
}

double* SysDynPtc::getState()
{
    double *state = new double[6 * m_objects.size()];
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
    }
    
    delete[] state;
    time = t;
    
    for (int i = 0; i < COLLISION_ITERATION; ++i) {
        m_objects.collid_detection(m_objects);
        m_objects.collid_detection(m_bounds);
    }
}

double* SysDynPtc::DerivEval(double* state, double t)
{
    m_objects.clearForce();
    ForceApply();
    m_objects.contact_detection(m_bounds);
    double *delta = new double[6 * m_objects.size()];
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
    }
    return delta;
}

void SysDynPtc::ForceApply()
{
    m_objects.ExcertForceField(ForceField::gravity);
    m_objects.ExcertForceField(ForceField::viscous);
    m_springs.ForceApply(ks, kd);
}

double SysDynPtc::getTime()
{
    return time;
}
