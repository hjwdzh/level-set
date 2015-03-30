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

RotJoint::RotJoint() {
}

RotJoint::RotJoint(const Vector3d& p1, const Vector3d& p2, const Vector3d& _tAxis, double min_angle, double max_angle) {
    pPos = p1;
    cPos = p2;
    tAxis = _tAxis;
    tAxis.normalize();
    this->min_angle = min_angle;
    this->max_angle = max_angle;
}

bool RotJoint::violated() {
    double ta = getAngle() - angle;
    if (ta < -180)
        ta += 360;
    if (ta > 180)
        ta -= 360;
    Vector3d d1 = parent->Transform() * dp;
    double dir = d1.dotProduct(wc - wp);
    if ((ta < min_angle && dir > 0) || (ta > max_angle && dir < 0)) {
        if (ta < min_angle)
            qt = Quatd(-tAxis * (min_angle / 180.0 * 3.141592654));
        else
            qt = Quatd(-tAxis * (max_angle / 180.0 * 3.141592654));
        return true;
    }
    return false;
}

void RotJoint::preStabilization(double h) {
    Vector3d pPos = parent->rotation.rotMatrix() * this->pPos;
    Vector3d cPos = child->rotation.rotMatrix() * this->cPos;
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
    if (!violated()) {
        if (!(parent->nailed)) {
            parent->v += j * term1;
            parent->w += J1 * pPos.crossProduct(j);
        }
        if (!(child->nailed)) {
            child->v -= j * term2;
            child->w -= J2 * cPos.crossProduct(j);
        }
    } else {
        Vector3d jt = solvejt(h);
        if (!(parent->nailed)) {
            parent->v += j * term1;
            parent->w += J1 * jt;
        }
        if (!(child->nailed)) {
            child->v -= j * term2;
            child->w -= J2 * jt;
        }
    }
}

double RotJoint::getAngle() {
    Vector3d a(0.7536, 0.4844, 0.3428);
    Vector3d pa = parent->rotation.rotMatrix() * a;
    Vector3d pc = child->rotation.rotMatrix() * a;
    Vector3d d1 = parent->Transform() * dp;
    Vector3d d2 = child->Transform() * dc;
    d1.normalize();
    d2.normalize();
    pa = pa - d1 * d1.dotProduct(pa);
    pc = pc - d2 * d2.dotProduct(pc);
    pa.normalize();
    pc.normalize();
    return atan2(pa.dotProduct(pc), d1.dotProduct(pa.crossProduct(pc))) / 3.141592654 * 180;
}

void RotJoint::initialize() {
    dp = parent->Inv_Transform() * tAxis;
    dc = child->Inv_Transform() * tAxis;
    dp.normalize();
    dc.normalize();
    angle = getAngle();
}

bool RotJoint::postStabilization() {
    Vector3d wrel;
    Vector3d tAxis;
    bool is_violated = false;
    if (!(parent->nailed && child->nailed)) {
        is_violated = true;
        Vector3d d1 = parent->Transform() * dp;
        Vector3d d2 = child->Transform() * dc;
        d1.normalize();
        d2.normalize();
        tAxis = d1;
        wp = d1 * d1.dotProduct(parent->w);
        wc = d2 * d2.dotProduct(child->w);
        if (!violated()) {
            wrel = wp - wc + child->w - parent->w;
            is_violated = false;
        } else {
            wrel = (wp - wc) * -kr + (child->w - parent->w);
        }
    } else {
        return false;
    }
    Vector3d pPos = parent->rotation.rotMatrix() * this->pPos;
    Vector3d cPos = child->rotation.rotMatrix() * this->cPos;
    Vector3d v1 = parent->v + parent->w.crossProduct(pPos);
    Vector3d v2 = child->v + child->w.crossProduct(cPos);
    Vector3d vrel = v2 - v1;
    if ((vrel.length() < 1e-4 && wrel.length() < 1e-4) || (parent->nailed && child->nailed))
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
    if (!is_violated) {
        Vector3d diff;
        Vector3d axis = wrel_A2.inverse() * tAxis;
        axis.normalize();
        while (true) {
            diff += axis * jt.dotProduct(axis);
            if (diff.length() < 1e-13)
                break;
            jt -= diff;
            diff = vrel_A1.inverse() * (vrel_A2 * diff);
            j += diff;
            diff = wrel_A2.inverse() * (wrel_A1 * diff);
        }
    }
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

void RotJoint::ExcertForce() {
    double da = (getAngle() - this->angle) * (3.141592654 / 180);
    Vector3d tAxis = parent->Transform() * dp;
    Vector3d ft = tAxis * (da * (-kh));
    double dir = (wc - wp).dotProduct(tAxis);
    if (dir > 0) {
        ft += tAxis * kf;
    }
    if (dir < 0) {
        ft -= tAxis * kf;
    }
    parent->ExcertMoment(ft);
    child->ExcertMoment(-ft);
}