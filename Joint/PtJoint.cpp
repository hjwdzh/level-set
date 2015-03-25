//
//  PtJoint.cpp
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#include "PtJoint.h"

PtJoint::PtJoint() {
}

PtJoint::PtJoint(const Vector3d& p1, const Vector3d& p2) {
    pPos = p1;
    cPos = p2;
}

bool PtJoint::violated() {
    return false;
}

void PtJoint::preStabilization(double h) {
    Vector3d j = solvej(h);
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
        parent->w += J1 * (parent->rotation.rotMatrix() * pPos).crossProduct(j);
    }
    if (!(child->nailed)) {
        child->v -= j * term2;
        child->w -= J2 * (child->rotation.rotMatrix() * cPos).crossProduct(j);
    }
}

bool PtJoint::postStabilization() {
    Vector3d pPos = parent->rotation.rotMatrix() * this->pPos;
    Vector3d cPos = child->rotation.rotMatrix() * this->cPos;
    Vector3d v1 = parent->v + parent->w.crossProduct(pPos);
    Vector3d v2 = child->v + child->w.crossProduct(cPos);
    Vector3d vrel = v2 - v1;
    if (vrel.length() < 1e-4 || (parent->nailed && child->nailed))
        return false;
    double term1 = parent->nailed ? 0 : 1 / parent->mass;
    double term2 = child->nailed ? 0 : 1 / child->mass;
    Matrix3d J1 = Matrix3d::createScale(0, 0, 0);
    Matrix3d J2 = J1;
    Matrix3d term3 = J1, term4 = J1;
    if (!(parent->nailed)) {
        Matrix3d rot1 = parent->rotation.rotMatrix();
        Matrix3d rot11 = Matrix3d::createCrossProductMatrix(pPos);
        J1 = rot1 * parent->J * rot1.transpose();
        term3 = rot11 * J1 * rot11.transpose();
    }
    if (!(child->nailed)) {
        Matrix3d rot2 = child->rotation.rotMatrix();
        Matrix3d rot22 = Matrix3d::createCrossProductMatrix(cPos);
        J2 = rot2 * child->J * rot2.transpose();
        term4 = rot22 * J2 * rot22.transpose();
    }
    Matrix3d k = Matrix3d() * (term1 + term2) + term3 + term4;
    Vector3d p = k.inverse() * vrel;
    if (!(parent->nailed)) {
        parent->v += p * term1;
        parent->w += J1 * pPos.crossProduct(p);
    }
    if (!(child->nailed)) {
        child->v -= p * term2;
        child->w -= J2 * cPos.crossProduct(p);
    }
    return true;
}

