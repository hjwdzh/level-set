//
//  RotJoint.cpp
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#include "RotJoint.h"
#include "ARRAY.h"
#include "Solver.h"

using namespace SimLib;

bool RotJoint::violated() {
    double ta = getAngle() - angle;
    if (ta < -180)
        ta += 360;
    if (ta > 180)
        ta -= 360;
    double dir = tAxis.dotProduct(wc - wp);
    return ((ta < min_angle && dir < 0) || (ta > max_angle && dir > 0));
}

void RotJoint::preStabilization() {
    
}

double RotJoint::getAngle() {
    Vector3d a(0.7536, 0.4844, 0.3428);
    Vector3d pa = parent->rotation.rotMatrix() * a;
    Vector3d pc = child->rotation.rotMatrix() * a;
    pa = tAxis * tAxis.dotProduct(pa);
    pc = tAxis * tAxis.dotProduct(pc);
    pa.normalize();
    pc.normalize();
    return atan2(pa.dotProduct(pc), tAxis.dotProduct(pa.crossProduct(pc))) / 3.141592654 * 180;
}

void RotJoint::initialize() {
    angle = getAngle();
    dp = parent->Inv_Transform() * tAxis;
    dc = child->Inv_Transform() * tAxis;
}

bool RotJoint::postStabilization() {
    Vector3d wrel;
    if (!(parent->nailed || child->nailed)) {
        Vector3d d1 = parent->Transform() * dp;
        Vector3d d2 = child->Transform() * dc;
        wp = d1 * d1.dotProduct(parent->w);
        wc = d2 * d2.dotProduct(child->w);
        if (!violated()) {
            wrel = tAxis * tAxis.dotProduct(wc - wp);
        }
    }
    Vector3d v1 = parent->v + parent->w.crossProduct(parent->Transform() * pPos);
    Vector3d v2 = child->v + child->w.crossProduct(child->Transform() * cPos);
    Vector3d vrel = v2 - v1;
    if ((vrel.length() < 1e-4  && wrel.length() < 1e-4) || (parent->nailed && child->nailed))
        return false;
    double term1 = parent->nailed ? 0 : 1 / parent->mass;
    double term2 = child->nailed ? 0 : 1 / child->mass;
    Matrix3d rot1 = parent->rotation.rotMatrix();
    Matrix3d rot2 = child->rotation.rotMatrix();
    Matrix3d J1 = Matrix3d::createScale(0, 0, 0);
    Matrix3d J2 = J1;
    if (!(parent->nailed)) {
        J1 = rot1 * parent->J * rot1.transpose();
    }
    if (!(child->nailed)) {
        J2 = rot2 * parent->J * rot2.transpose();
    }
    Matrix3d rp = Matrix3d::createCrossProductMatrix(pPos);
    Matrix3d rc = Matrix3d::createCrossProductMatrix(cPos);
    Matrix3d rpt = rp.transpose();
    Matrix3d rct = rp.transpose();
    Matrix3d vrel_A1 = Matrix3d() * (term1 + term2) + rpt * J1 * rp + rct * J2 * rc;
    Matrix3d vrel_A2 = rpt * J1 + rct * J2;
    Matrix3d wrel_A1 = J1 * rp + J2 * rc;
    Matrix3d wrel_A2 = J1 + J2;
    ARRAY<2, double> a(6,6);
    ARRAY<1, double> b(6);
    ARRAY<1, double> x(6);
    for (int i = 1; i <= 3; ++i) {
        b(i) = vrel[i-1];
        b(i + 3) = wrel[i-1];
        for (int j = 1; j <= 3; ++j) {
            a(i,j) = vrel_A1.at(j,i);
            a(i,j+3) = vrel_A2.at(j,i);
            a(i+3,j) = wrel_A1.at(j,i);
            a(i+3,j+3) = wrel_A2.at(j,i);
        }
    }
    Solver::LinearSolve(a, b, x);
    Vector3d j(x(1), x(2), x(3));
    Vector3d jt(x(4), x(5), x(6));
    if (!(parent->nailed)) {
        parent->v += j * term1;
        parent->w += J1 * (pPos.crossProduct(j) + jt);
    }
    if (!(child->nailed)) {
        child->v -= j * term2;
        child->w -= J2 * (cPos.crossProduct(j) + jt);
    }
    return true;
}
