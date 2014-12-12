//
//  Particles.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "main.h"
#include "Particle.h"
#include "vmath.h"

Particle::Particle()
{}

Particle::Particle(const Vector3d &_x, double _m)
{
    x = _x;
    mass = _m;
}

Particle::Particle(const Vector3d &_x, const Vector3d &_v, double _m)
{
    x = _x;
    v = _v;
    mass = _m;
}

double Particle::getMouseDepth(double mouseX, double mouseY)
{
    Vector4d v(x[0], x[1], x[2], 1);
    v = g_camera->lookat * v;
    v = v / v[3];
    double dx = (v[0] / v[2] * (-g_d) - mouseX) / g_right * g_WindowWidth * 0.5;
    double dy = (v[1] / v[2] * (-g_d) - mouseY) / g_top * g_WindowHeight * 0.5;
    if (sqrt(dx * dx + dy * dy) < 10)
    {
        return v[2];
    }
    return 1;
}

void Particle::ExcertForce(const Vector3d &force)
{
    if (!nailed)
        f += force;
}

void Particle::ExcertForceField(Vector3d (*forcefunc)(Geometric*))
{
    if (!nailed)
        f += forcefunc(this);
}

void Particle::Display()
{
    if (nailed)
        glColor3f(0.0f, 0.0f, 1.0f);
    else
    if (selected)
        glColor3f(1.0f, 0.0f, 1.0f);
    else
        glColor3f(0.0, 1.0f, 0.0f);
    glPushMatrix();
    glTranslated(x[0], x[1], x[2]);
    glutSolidSphere(0.03f, 15, 15);
    glScaled(0.2, 0.2, 0.2);
    if (extForce.length() > 0)
    {
        glColor3f(1.0f, 0.0, 0.0);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(extForce[0], extForce[1], extForce[2]);
        glEnd();
        glTranslated(extForce[0], extForce[1], extForce[2]);
        glutSolidSphere(0.01f, 15, 15);
        glTranslated(-extForce[0], -extForce[1], -extForce[2]);
    }
    if (userForce.length() > 0)
    {
        glColor3f(1.0f, 1.0, 1.0);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0);
        glVertex3d(userForce[0], userForce[1], userForce[2]);
        glEnd();
        glTranslated(userForce[0], userForce[1], userForce[2]);
        glutSolidSphere(0.01f, 15, 15);
    }
    glPopMatrix();
}

