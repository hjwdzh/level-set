//
//  PtJoint.cpp
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#include "PtJoint.h"
#include <sys/time.h>
extern double g_jointTime;

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
    timeval t1, t2;
    gettimeofday(&t1, 0);
    Vector3d j = solvej(h);
    Matrix3d& J1 = parent->Jr;
    Matrix3d& J2 = child->Jr;
/*
    if (!(parent->nailed)) {
        Matrix3d rot1 = parent->rotation.rotMatrix();
        J1 = rot1 * parent->J * rot1.transpose();
    }
    if (!(child->nailed)) {
        Matrix3d rot2 = child->rotation.rotMatrix();
        J2 = rot2 * child->J * rot2.transpose();
    }
*/    double term1 = parent->nailed ? 0 : 1 / parent->mass;
    double term2 = child->nailed ? 0 : 1 / child->mass;
    if (!(parent->nailed)) {
        parent->v += j * term1;
        parent->w += J1 * (parent->rotation.rotMatrix() * pPos).crossProduct(j);
    }
    if (!(child->nailed)) {
        child->v -= j * term2;
        child->w -= J2 * (child->rotation.rotMatrix() * cPos).crossProduct(j);
    }
    gettimeofday(&t2, 0);
//    g_jointTime += (t2.tv_usec - t1.tv_usec) * 1e-6 + (t2.tv_sec - t1.tv_sec);
}

bool PtJoint::postStabilization() {
    timeval t1, t2;
    gettimeofday(&t1, 0);
    Vector3d pPos = parent->rotation.rotMatrix() * this->pPos;
    Vector3d cPos = child->rotation.rotMatrix() * this->cPos;
    Vector3d v1 = parent->v + parent->w.crossProduct(pPos);
    Vector3d v2 = child->v + child->w.crossProduct(cPos);
    Vector3d vrel = v2 - v1;
    if (vrel.length() < 1e-4 || (parent->nailed && child->nailed)) {
        gettimeofday(&t2, 0);
//        g_jointTime += (t2.tv_sec - t1.tv_sec) + 1e-6 * (t2.tv_usec - t1.tv_usec);
        return false;
    }
    double term1 = parent->nailed ? 0 : 1 / parent->mass;
    double term2 = child->nailed ? 0 : 1 / child->mass;
    Matrix3d& J1 = parent->Jr;
    Matrix3d& J2 = child->Jr;
    Matrix3d term3 = Matrix3d::createScale(0, 0, 0);
    Matrix3d term4 = term3;
    if (!(parent->nailed)) {
        Matrix3d rot11 = Matrix3d::createCrossProductMatrix(pPos);
        term3 = rot11 * J1 * rot11.transpose();
    }
    if (!(child->nailed)) {
        Matrix3d rot22 = Matrix3d::createCrossProductMatrix(cPos);
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
    gettimeofday(&t2, 0);
//    g_jointTime += (t2.tv_sec - t1.tv_sec) + 1e-6 * (t2.tv_usec - t1.tv_usec);

    return true;
}

