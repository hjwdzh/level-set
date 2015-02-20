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

PtJoint::PtJoint(Vector3d p1, Vector3d p2) {
    pPos = p1;
    cPos = p2;
}

bool PtJoint::violated() {
    return false;
}

void PtJoint::preStabilization() {
    
}

bool PtJoint::postStabilization() {
    Vector3d v1 = parent->v + parent->w.crossProduct(pPos);
    Vector3d v2 = child->v + child->w.crossProduct(cPos);
    Vector3d vrel = v2 - v1;
    if (vrel.length() < 1e-4 || (parent->nailed && child->nailed))
        return false;
    double numerator = 1;
    double term1 = parent->nailed ? 0 : 1 / parent->mass;
    double term2 = child->nailed ? 0 : 1 / child->mass;
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
    Vector3d n = vrel;
    n.normalize();
    double term3 = n.dotProduct((J1 * (pPos.crossProduct(n))).crossProduct(pPos));
    double term4 = n.dotProduct((J2 * (cPos.crossProduct(n))).crossProduct(cPos));
    double j = numerator / (term1 + term2 + term3 + term4);
    Vector3d p = vrel * j;
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

