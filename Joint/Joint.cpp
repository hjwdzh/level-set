//
//  Joint.cpp
//  levelset
//
//  Created by Jingwei Huang on 15-2-13.
//  Copyright (c) 2015年 Jingwei Huang. All rights reserved.
//

#include "Joint.h"
#include "ARRAY.h"
#include "Solver.h"

using namespace SimLib;

bool Joint::violated() {
    return false;
}

void Joint::initialize() {
    
}

void Joint::preStabilization() {
    
}

bool Joint::postStabilization() {
    Vector3d v1 = parent->v + parent->w.crossProduct(pPos);
    Vector3d v2 = child->v + child->w.crossProduct(cPos);
    Vector3d vrel = v2 - v1;
    Vector3d wrel = child->w - parent->w;
    if ((vrel.length() < 1e-4 && wrel.length() < 1e-4) || (parent->nailed && child->nailed))
        return false;
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
    Matrix3d rp = Matrix3d::createCrossProductMatrix(parent->Transform() * pPos);
    Matrix3d rc = Matrix3d::createCrossProductMatrix(child->Transform() * cPos);
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
    
    return false;
}

Vector3d Joint::f(double h, Vector3d& j) {
    double mp = parent->nailed ? 0 : 1 / parent->mass;
    double mc = child->nailed ? 0 : 1 / child->mass;
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
    Vector3d rp0 = parent->rotation.rotMatrix() * pPos;
    Vector3d rc0 = child->rotation.rotMatrix() * cPos;
    return j * (h * (mp + mc)) + (parent->v * h + (Quatd(parent->w * h + (J1 * rp0.crossProduct(j)) * h) * parent->rotation).rotMatrix() * pPos) - (child->v * h + (Quatd(child->w * h - (J2 * rc0.crossProduct(j)) * h) * child->rotation).rotMatrix() * cPos);
}

Quatd Joint::ft(double h, Vector3d& jt) {
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
    return (Quatd(parent->w * h + J1 * jt * h) * parent->rotation) - (Quatd(child->w * h - J2 * jt * h) * child->rotation);
}

void Joint::dfj(double h, const Vector3d &j, Matrix3d &r) {
    Vector3d rp0 = parent->rotation.rotMatrix() * pPos;
    Vector3d rc0 = child->rotation.rotMatrix() * cPos;
    Matrix3d rp = Matrix3d::createCrossProductMatrix(rp0);
    Matrix3d rc = Matrix3d::createCrossProductMatrix(rc0);
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
    Vector3d wp0 = (parent->w + J1 * (rp * j)) * h;
    Vector3d wc0 = (child->w - J2 * (rc * j)) * h;
    double thetap = wp0.length() * 0.5;
    double thetac = wc0.length() * 0.5;
    Vector3d wp = wp0 * (1 / wp0.length());
    Vector3d wc = wc0 * (1 / wc0.length());
    Vector3d dthetap = (J1 * rp).transpose() * (wp * (h * 0.5));
    Vector3d dthetac = (J2 * rc).transpose() * (wc * (-h * 0.5));
    Matrix3d dwp = (Matrix3d() - Matrix3d::createDotProductMatrix(wp, wp)) * J1 * rp * (h / (2 * thetap));
    Matrix3d dwc = (Matrix3d() - Matrix3d::createDotProductMatrix(wc, wc)) * J2 * rc * (-h / (2 * thetac));
    Vector3d t1 = rp0 * (cos(thetap) * sin(thetap)) - rp * wp * (2 * cos(thetap) * cos(thetap)) + rp * wp * (2 * sin(thetap) * sin(thetap)) + wp * (rp0.dotProduct(wp) * 4 * sin(thetap) * cos(thetap));
    Matrix3d t2 = rp * (-2 * sin(thetap) * cos(thetap)) + Matrix3d() * (2 * rp0.dotProduct(wp) * sin(thetap) * sin(thetap)) + Matrix3d::createDotProductMatrix(wp * (2 * sin(thetap) * sin(thetap)), rp0);
    Matrix3d resp = Matrix3d::createDotProductMatrix(t1, dthetap) + t2 * dwp;
    t1 = rc0 * (cos(thetac) * sin(thetac)) - rc * wc * (2 * cos(thetac) * cos(thetac)) + rc * wc * (2 * sin(thetac) * sin(thetac)) + wc * (rc0.dotProduct(wc) * 4 * sin(thetac) * cos(thetac));
    t2 = rc * (-2 * sin(thetac) * cos(thetac)) + Matrix3d() * (2 * rc0.dotProduct(wc) * sin(thetac) * sin(thetac)) + Matrix3d::createDotProductMatrix(wc * (2 * sin(thetac) * sin(thetac)), rc0);
    Matrix3d resc = Matrix3d::createDotProductMatrix(t1, dthetac) + t2 * dwc;
    double m1 = parent->nailed ? 0 : 1 / parent->mass;
    double m2 = child->nailed ? 0 : 1 / child->mass;
    r = Matrix3d() * (h * (m1 + m2)) + resp - resc;
}

void Joint::dfjt(double h, const Vector3d& jt, std::pair<Vector3d,Matrix3d>& r) {
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
    Vector3d wp0 = (parent->w + J1 * jt) * h;
    Vector3d wc0 = (child->w - J2 * jt) * h;
    double thetap = wp0.length() * 0.5;
    double thetac = wc0.length() * 0.5;
    Vector3d wp = wp0 * (1 / wp0.length());
    Vector3d wc = wc0 * (1 / wc0.length());
    Vector3d dthetap = J1.transpose() * (wp * (h * 0.5));
    Vector3d dthetac = J2.transpose() * (wc * (-h * 0.5));
    Matrix3d dwp = (Matrix3d() - Matrix3d::createDotProductMatrix(wp, wp)) * J1 * (h / (2 * thetap));
    Matrix3d dwc = (Matrix3d() - Matrix3d::createDotProductMatrix(wc, wc)) * J2 * (-h / (2 * thetac));
    Vector3d ap = dthetap * (-parent->rotation.w * sin(thetap) - parent->rotation.v.dotProduct(wp) * cos(thetap))
            - dwp.transpose() * parent->rotation.v;
    Vector3d ac = dthetac * (-child->rotation.w * sin(thetac) - child->rotation.v.dotProduct(wc) * cos(thetac))
            - dwc.transpose() * child->rotation.v;
    r.first = ap - ac;
    Vector3d t1 = parent->rotation.v * (-sin(thetap)) + wp * (parent->rotation.w * cos(thetap)) - parent->rotation.v.crossProduct(wp) * cos(thetap);
    Matrix3d t2 = Matrix3d() * (parent->rotation.w * sin(thetap)) - Matrix3d::createCrossProductMatrix(parent->v) * sin(thetap);
    Matrix3d resp = Matrix3d::createDotProductMatrix(dthetap, t1) + t2 * dwp;
    t1 = child->rotation.v * (-sin(thetac)) + wc * (child->rotation.w * cos(thetac)) - child->rotation.v.crossProduct(wc) * cos(thetac);
    t2 = Matrix3d() * (child->rotation.w * sin(thetac)) - Matrix3d::createCrossProductMatrix(child->v) * sin(thetac);
    Matrix3d resc = Matrix3d::createDotProductMatrix(dthetap, t1) + t2 * dwp;
    r.second = resp - resc;
}

Vector3d Joint::solvej(double h) {
    Vector3d j, dj;
    Matrix3d dfjval;
    Vector3d fval = f(h, j);
    double epsilon = 0.1;
    while (fval.length() > 1e-6) {
        dfj(h, j, dfjval);
        dj = dfjval.inverse() * fval;
        j += dj * epsilon;
        if (epsilon < 1 - 1e-6)
            epsilon += 0.05;
        fval = f(h, j);
    }
    return fval;
}

Vector3d Joint::solvejt(double h) {
    return Vector3d();
}