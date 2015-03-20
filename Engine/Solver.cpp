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

void Solver::NRBS(SystemPhy &sys, double h) {
    sys.setSolver(SystemPhy::NRBS);
    double t = sys.getTime();
    int n = sys.getDim() / 13 * 6;
    int m = sys.getDim() / 13 * 7;
    //collide detection
    //use x_new = x + h * v_new and v_new = v + dv to test collision
    double* v = sys.getVelState();
    double* deltaV = sys.DerivVelEval(v, t);
    double* v_new = new double[n];
    if (deltaV != 0) {
        for (int i = 0; i < n; ++i) {
            v_new[i] = v[i] + deltaV[i] * h;
        }
    }
    double* x = sys.getPosState();
    sys.setVelState(v_new, t);
    double* deltaX = sys.DerivPosEval(x, t);
    double* x_new = new double[m];
    if (deltaX != 0) {
        for (int i = 0; i < m; ++i) {
            if (i % 7 < 3) {
                x_new[i] = x[i] + h * deltaX[i];
            }
            else if (i % 7 == 3) {
                Quaternion<double> rotation(x[i+3],x[i],x[i+1],x[i+2]);
                Quaternion<double> rotation1 = Quaternion<double>::fromEulerAngles(h*deltaX[i]*ANGLE_SCALE, h*deltaX[i+1]*ANGLE_SCALE, h*deltaX[i+2]*ANGLE_SCALE);
                rotation = rotation1 * rotation;
                x_new[i] = rotation.v[0];
                x_new[i+1] = rotation.v[1];
                x_new[i+2] = rotation.v[2];
                x_new[i+3] = rotation.w;
            }
        }
    }
    sys.setPosState(x_new, t);
    sys.setVelState(v, t);
    sys.collide_detection();
    sys.postStabilization();
    delete[] v;
    delete[] deltaV;
    delete[] v_new;
    delete[] deltaX;
    
    //integrate velocity
    v = sys.getVelState();
    deltaV = sys.DerivVelEval(v, t);
    if (deltaV != 0) {
        for (int i = 0; i < n; ++i) {
            v[i] += deltaV[i] * h;
        }
    }
    sys.setVelState(v, t);
    sys.postStabilization();
    delete[] v;
    delete[] deltaV;

    sys.preStabilization();
    //contact handling
    //use x_new = x + h * v
    deltaX = sys.DerivPosEval(x, t);
    if (deltaX != 0) {
        for (int i = 0; i < m; ++i) {
            if (i % 7 < 3) {
                x_new[i] = x[i] + h * deltaX[i];
            }
            else if (i % 7 == 3) {
                Quaternion<double> rotation(x[i+3],x[i],x[i+1],x[i+2]);
                Quaternion<double> rotation1 = Quaternion<double>::fromEulerAngles(h*deltaX[i]*ANGLE_SCALE, h*deltaX[i+1]*ANGLE_SCALE, h*deltaX[i+2]*ANGLE_SCALE);
                rotation = rotation1 * rotation;
                x_new[i] = rotation.v[0];
                x_new[i+1] = rotation.v[1];
                x_new[i+2] = rotation.v[2];
                x_new[i+3] = rotation.w;
            }
        }
    }
    sys.setPosState(x_new, t);
    sys.contact_handling();
    delete[] deltaX;
    
    //Integrate x
    deltaX = sys.DerivPosEval(x, t);
    if (deltaX != 0) {
        for (int i = 0; i < m; ++i) {
            if (i % 7 < 3) {
                x_new[i] = x[i] + h * deltaX[i];
            }
            else if (i % 7 == 3) {
                Quaternion<double> rotation(x[i+3],x[i],x[i+1],x[i+2]);
                Quaternion<double> rotation1 = Quaternion<double>::fromEulerAngles(h*deltaX[i]*ANGLE_SCALE, h*deltaX[i+1]*ANGLE_SCALE, h*deltaX[i+2]*ANGLE_SCALE);
                rotation = rotation1 * rotation;
                x_new[i] = rotation.v[0];
                x_new[i+1] = rotation.v[1];
                x_new[i+2] = rotation.v[2];
                x_new[i+3] = rotation.w;
            }
        }
    }
    sys.setPosState(x_new, t + h);
    
    delete[] x;
    delete[] x_new;
    delete[] deltaX;
    sys.updateForce();
}


void Solver::EulersStep(SystemPhy &sys, double h)
{
    sys.setSolver(SystemPhy::LCP);
    double t = sys.getTime();
    int n = sys.getDim();
    double* x = sys.getState();
    double* deltaX = sys.DerivEval(x, t);
    if (deltaX != 0) {
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
    }
    sys.setState(x, h);
    sys.updateForce();
    sys.collide_detection();
    sys.contact_handling();
}

void Solver::QPSolve(ARRAY<2, double>& a, ARRAY<1, double>& b, ARRAY<1, double>& f) {
    int n = f.dim(1);
    f.Fill(0);
    ARRAY<1, double> c = b;
    ARRAY<1, double> delta_f(n), delta_a(n);
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
        NC.erase(d);
        while (true) {
            fdirection(d, a, C, delta_f, delta_a);
            pair<double, int> sj = maxstep(f,c,delta_f,delta_a,C,NC,d);
            if (sj.first > 1e20) {
                return;
            }
            for (set<int>::iterator it = C.begin(); it != C.end(); ++it)
                if (*it != d)
                    f(*it) += sj.first * delta_f(*it);
            f(d) += sj.first * delta_f(d);
            for (int i = 1; i <= n; ++i) {
                c(i) += delta_a(i) * sj.first;
                if (c(i) < -1e-4) {
                    ad_minus.insert(i);
                } else {
                    if (i == d) {
                        sj.second = d;
                        NC.erase(d);
                    }
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
    for (int i = 1; i <= n; ++i) {
        double t = 0;
        for (int j = 1; j <= n; ++j) {
            t += a(i,j) * delta_f(j);
        }
    }
}

// Ax = b
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

//http://m.oschina.net/blog/3758
void Solver::SVD(SimLib::ARRAY<2, double> a, SimLib::ARRAY<2, double>& u, SimLib::ARRAY<1, double>& s, SimLib::ARRAY<2, double>& v) {
    int m = a.dim(1);
    int n = a.dim(2);
    if (m < n) {
        ARRAY<2, double> A(n, m);
        for (int i = 1; i <= m; ++i) {
            for (int j = 1; j <= n; ++j) {
                A(j,i) = a(i,j);
            }
        }
        SVD(A, v, s, u);
        return;
    }
    u = ARRAY<2, double>(m,m);
    s = ARRAY<1, double>(n);
    v = ARRAY<2, double>(n,n);
    ARRAY<1, double> e(n);
    ARRAY<1, double> work(m);
    int wantu = 1;
    int wantv = 1;
    int nct = m - 1;
    int nrt = std::max(0, n - 2);
    int i = 0, j = 0, k = 0;
    for (k = 1; k <= std::max(nct, nrt); ++k) {
        if (k <= nct) {
            s(k) = 0;
            for (i = k; i <= m; ++i)
                s(k) = hypot(s(k), a(i,k));
            if (s(k) < -1e-6 || s(k) > 1e-6) {
                if (a(k,k) < 0)
                    s(k) = -s(k);
                for (i = k; i <= m; ++i)
                    a(i,k) /= s(k);
                a(k,k) += 1;
            }
            s(k) = -s(k);
        }
        for (j = k + 1; j <= n; ++j) {
            if (k <= nct && (s(k) < -1e-6 || s(k) > 1e-6)) {
                double t = 0;
                for (i = k; i <= m; ++i)
                    t += a(i,k) * a(i,j);
                t = -t / a(k,k);
                for (i = k; i <= m; ++i)
                    a(i,j) += t * a(i,k);
            }
            e(k) = a(k,j);
        }
        if (wantu & (k <= nct)) {
            for (i = k; i <= m; ++i)
                u(i,k) = a(i,k);
        }
        if (k <= nrt) {
            e(k) = 0;
            for (i = k + 1; i <= n; ++i)
                e(k) = hypot(e(k), e(i));
            if (e(k) > 1e-6 || e(k) < -1e-6) {
                if (e(k+1) < 0)
                    e(k) = -e(k);
                for (i = k + 1; i <= n; ++i)
                    e(i) /= e(k);
                e(k + 1) += 1;
            }
            e(k) = -e(k);
            if ((k + 1 < m) && (e(k) > 1e-6 || e(k) < -1e-6)) {
                for (i = k + 1; i <= m; ++i)
                    work(i) = 0;
                for (j = k + 1; j <= n; ++j) {
                    for (i = k + 1; i <= m; ++i) {
                        work(i) += e(j) * a(i,j);
                    }
                }
                for (j = k + 1; j <= n; ++j) {
                    double t = -e(j) / e(k + 1);
                    for (int i = k + 1; i <= m; ++i)
                        a(i,j) += t * work(i);
                }
            }
            if (wantv) {
                for (i = k + 1; i <= n; ++i)
                    v(i,k) = e(i);
            }
        }
    }
    int p = n;
    if (nct < n) {
        s(nct + 1) = a(nct + 1,nct + 1);
    }
    if (m < p) {
        s(p) = 0;
    }
    if (nrt + 1 < p) {
        e(nrt + 1) = a(nrt + 1, p);
    }
    e(p) = 0;
    if (wantu) {
        for (j = nct + 1; j <= n; ++j) {
            for (i = 1; i <= m; ++i)
                u(i,j) = 0;
            u(j,j) = 1;
        }
        for (k = nct; k >= 1; --k) {
            if (s(k) > 1e-6 || s(k) < -1e-6) {
                for (j = k + 1; j <= n; ++j) {
                    double t = 0;
                    for (i = k; i <= m; ++i)
                        t += u(i,k) * u(i,j);
                    t = -t / u(k,k);
                    for (i = k; i <= m; ++i)
                        u(i,j) += t * u(i,k);
                }
                for (i = k; i <= m; ++i) {
                    u(i,k) = -u(i,k);
                }
                u(k,k) += 1;
                for (i = 1; i <= k - 1; ++i) {
                    u(i,k) = 0;
                }
            } else {
                for (i = 1; i <= m; ++i)
                    u(i,k) = 0;
                u(k,k) = 1;
            }
        }
    }
    if (wantv) {
        for (k = n; k >= 1; --k) {
            if ((k <= nrt) && (e(k) > 1e-6 || e(k) < -1e-6)) {
                for (j = k + 1; j <= n; ++j) {
                    double t = 0;
                    for (i = k + 1; i <= n; ++i)
                        t += v(i,k) * v(i,j);
                    t = -t / v(k+1, k);
                    for (i = k + 1; i <= n; ++i)
                        v(i,j) += t * v(i,k);
                }
            }
            for (i = 1; i <= n; ++i) {
                v(i,k) = 0;
            }
            v(k,k) = 1;
        }
    }
}

//min ||Ax - b||
void Solver::OLS(SimLib::ARRAY<2, double>& a, SimLib::ARRAY<1, double>& b, SimLib::ARRAY<1, double>& x) {
    ARRAY<2, double> u, v;
    ARRAY<1, double> s;
    SVD(a, u, s, v);
    ARRAY<1, double> b1(4);
    for (int i = 1; i <= 4; ++i) {
        for (int j = 1; j <= 4; ++j) {
            b1(i) += u(j,i) * b(j);
        }
    }
    for (int i = 1; i <= 3; ++i)
        b1(i) /= s(i);
    x = ARRAY<1, double>(3);
    for (int i = 1; i <= 3; ++i) {
        for (int j = 1; j <= 3; ++j) {
            x(i) += v(i,j) * b1(j);
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