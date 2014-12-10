//
//  Solver.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Solver.h"
#include "vmath.h"
#include <set>
#include <list>
#define ANGLE_SCALE 180 / 3.141592654

using namespace SimLib;

void Solver::EulersStep(SystemPhy &sys, double h)
{
    double t = sys.getTime();
    int n = sys.getDim();
    double* x = sys.getState();
    double* deltaX = sys.DerivEval(x, t);
    for (int i = 0; i < n; ++i)
    {
        if (i % 13 < 6 || i % 13 >= 10)
            x[i] += h * deltaX[i];
        else if (i % 13 == 6){
            Quaternion<double> rotation(x[i+3],x[i],x[i+1],x[i+2]);
            Quaternion<double> rotation1 = Quaternion<double>::fromEulerAngles(h*deltaX[i]*ANGLE_SCALE, h*deltaX[i+1]*ANGLE_SCALE, h*deltaX[i+2]*ANGLE_SCALE);
            rotation = rotation1 * rotation;
            x[i] = rotation.v[0];
            x[i+1] = rotation.v[1];
            x[i+2] = rotation.v[2];
            x[i+3] = rotation.w;
        }
    }
    sys.setState(x, t + h);
}

void Solver::QPSolve(ARRAY<2, double>& a, ARRAY<1, double>& b, ARRAY<1, double>& f) {
    int n = f.dim(1);
    f.Fill(0);
    ARRAY<1, double> c = b, delta_f(n), delta_a(n);
    set<int> C, NC;
    set<int> ad_minus;
    for (int i = 1; i <= n; ++i) {
        if (c(i) < -1e-4) {
            ad_minus.insert(i);
        }
    }
    
    while (!ad_minus.empty()) {
        set<int>::iterator it = ad_minus.begin();
        int d = *it;
        while (true) {
            fdirection(d, a, C, delta_f, delta_a);
            pair<double, int> sj = maxstep(f,c,delta_f,delta_a,C,NC,d);
            for (set<int>::iterator it = C.begin(); it != C.end(); ++it)
                if (*it != d)
                    f(*it) += sj.first * delta_f(*it);
            f(d) += sj.first * delta_f(d);
            for (int i = 1; i <= n; ++i) {
                c(i) += delta_a(i) * sj.first;
                if (c(i) < -1e-4) {
                    ad_minus.insert(i);
                } else {
                    ad_minus.erase(i);
                }
            }
            it = C.find(sj.second);
            if (it != C.end()) {
                NC.insert(*it);
                C.erase(it);
            } else {
                it = NC.find(sj.second);
                if (it != NC.end()) {
                    C.insert(*it);
                    NC.erase(it);
                } else {
                    C.insert(d);
                    ad_minus.erase(d);
                    break;
                }
            }
        }
    }
}

void Solver::fdirection(int d, ARRAY<2, double>& a, set<int> &C, ARRAY<1, double> &delta_f, ARRAY<1, double> &delta_a) {
    int n = delta_f.dim(1);
    for (int i = 1; i <= n; ++i) {
        delta_a(i) = a(i,d);
        delta_f(i) = 0;
    }
    delta_f(d) = 1;
    bool flag = false;
    for (int i = 1; i <= n; ++i) {
        double ttt = 0;
        for (int j = 1; j <= n; ++j)
            ttt += a(i,j) * delta_f(j);
        if (fabs(ttt - delta_a(i)) > 1e-4) {
            flag = true;
            break;
        }
    }
    if (!C.empty()) {
        ARRAY<1, double> vi((int)C.size()), X((int)C.size());
        ARRAY<2, double> A((int)C.size(), (int)C.size());
        int s = 0;
        for (set<int>::iterator it = C.begin(); it != C.end(); ++it) {
            s++;
            int s1 = 0;
            for (set<int>::iterator it1 = C.begin(); it1 != C.end(); ++it1) {
                s1++;
                A(s,s1) = a(*it,*it1);
            }
            vi(s) = -a(*it, d);
        }
        LinearSolve(A, vi, X);
        s = 0;
        for (set<int>::iterator it = C.begin(); it != C.end(); ++it) {
            s++;
            delta_f(*it) = X(s);
            for (int i = 1; i <= n; ++i) {
                delta_a(i) += a(i,*it) * X(s);
            }
        }
    }
}

void Solver::LinearSolve(ARRAY<2, double> a, ARRAY<1, double> b, ARRAY<1, double>& x) {
    int n = a.dim(1);
    for (int i = 1; i <= n; ++i) {
        int j = i;
        while (j <= n && a(j,i) == 0)
            ++j;
        if (j > n) {
            continue;
        }
        if (j != i) {
            for (int k = i; k <= n; ++k)
                swap(a(j,k), a(i,k));
            swap(b(j), b(i));
        }
        for (int k = j + 1; k <= n; ++k) {
            if (a(k,i) != 0) {
                double w = a(k,i) / a(i,i);
                for (int l = i; l <= n; ++l) {
                    a(k,l) -= w * a(i,l);
                }
                b(k) -= w * b(i);
            }
        }
    }
    for (int i = n; i >= 1; --i) {
        x(i) = b(i) / a(i,i);
        for (int j = i - 1; j >= 1; --j) {
            b(j) -= x(i) * a(j,i);
        }
    }
}

pair<double, int> Solver::maxstep(SimLib::ARRAY<1, double>& f, SimLib::ARRAY<1, double>& a, SimLib::ARRAY<1, double>& delta_f, SimLib::ARRAY<1, double>& delta_a, set<int>& C, set<int>& NC, int d) {
    double s = 1e30;
    int j = -1;
    if (delta_a(d) > 0) {
        j = d;
        s = -a(d) / delta_a(d);
    }
    for (set<int>::iterator it = C.begin(); it != C.end(); ++it) {
        if (delta_f(*it) < -1e-4) {
            double s1 = -f(*it) / delta_f(*it);
            if (s1 < s) {
                s = s1;
                j = *it;
            }
        }
    }
    for (set<int>::iterator it = NC.begin(); it != NC.end(); ++it) {
        if (delta_a(*it) < -1e-4) {
            double s1 = -a(*it) / delta_a(*it);
            if (s1 < s) {
                s = s1;
                j = *it;
            }
        }
    }
    return make_pair(s,j);
}