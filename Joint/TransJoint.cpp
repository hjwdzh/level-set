//
//  TransJoint.cpp
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#include "TransJoint.h"
#include "ARRAY.h"
#include "Solver.h"

using namespace SimLib;

TransJoint::TransJoint() {
}

TransJoint::TransJoint(const Vector3d& p1, const Vector3d& p2, const Vector3d& _tAxis, double _min_dis, double _max_dis) {
    pPos = p1;
    cPos = p2;
    tAxis = _tAxis;
    tAxis.normalize();
    min_dis = _min_dis;
    max_dis = _max_dis;
    qt = Quatd(Vector3d());
}

bool TransJoint::violated() {
    Vector3d pPos = parent->rotation.rotMatrix() * this->pPos;
    Vector3d cPos = child->rotation.rotMatrix() * this->cPos;
    vp = parent->v + parent->w.crossProduct(pPos);
    vc = child->v + child->w.crossProduct(cPos);
    Vector4d curDis = dis - getDistance();
    double dis = curDis[0] * tAxis[0] + curDis[1] * tAxis[1] + curDis[2] * tAxis[2];
    double dir = (vc - vp).dotProduct(tAxis);
    return ((dis < min_dis && dir < 0) || (dis > max_dis && dir > 0));
}

void TransJoint::preStabilization(double h) {
    Vector3d j = solvej(h);
    Vector3d jt = solvejt(h);
    if (!violated()) {
        j -= tAxis * j.dotProduct(tAxis);
    }
    Matrix3d J1 = Matrix3d::createScale(0, 0, 0);
    Matrix3d J2 = J1;
    if (!(parent->nailed)) {
        Matrix3d rot1 = parent->rotation.rotMatrix();
        J1 = rot1 * parent->J * rot1.transpose();
    }
    if (!(child->nailed)) {
        Matrix3d rot2 = child->rotation.rotMatrix();
        J2 = rot2 * child->J * rot2.transpose();
    }
    double term1 = parent->nailed ? 0 : 1 / parent->mass;
    double term2 = child->nailed ? 0 : 1 / child->mass;
    if (!(parent->nailed)) {
        parent->v += j * term1;
        parent->w += J1 * jt;
    }
    if (!(child->nailed)) {
        child->v -= j * term2;
        child->w -= J2 * jt;
    }
}

bool TransJoint::postStabilization() {
    Vector3d vrel;
    Vector3d wrel = child->w - parent->w;
    if (!violated()) {
        vrel = vc - vp;
        vrel -= tAxis * vrel.dotProduct(tAxis);
    } else {
        vrel = vc - vp;
    }
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
        J2 = rot2 * child->J * rot2.transpose();
    }
    Matrix3d rp = Matrix3d::createCrossProductMatrix(pPos);
    Matrix3d rc = Matrix3d::createCrossProductMatrix(cPos);
    Matrix3d rpt = rp.transpose();
    Matrix3d rct = rc.transpose();
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
            a(i,j) = vrel_A1.at(j-1,i-1);
            a(i,j+3) = vrel_A2.at(j-1,i-1);
            a(i+3,j) = wrel_A1.at(j-1,i-1);
            a(i+3,j+3) = wrel_A2.at(j-1,i-1);
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

Vector4d TransJoint::getDistance() {
    return (parent->Transform() * pX - child->Transform() * cX);
}

void TransJoint::initialize() {
    pX = parent->Inv_Transform() * Vector4d(pPos[0], pPos[1], pPos[2], 1);
    cX = child->Inv_Transform() * Vector4d(cPos[0], cPos[1], cPos[2], 1);
    dis = getDistance();
}