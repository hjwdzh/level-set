//
//  Solver.cpp
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Solver.h"
#include "vmath.h"
#include "SysDynPtc.h"
#include <set>
#include <list>
#include <sys/time.h>
#define ANGLE_SCALE 180 / 3.141592654
using namespace SimLib;
extern double g_simTime;
double g_jointTime;

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
    g_jointTime = 0;
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

 //   delete[] deltaX;

    sys.preStabilization(h);
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
    sys.postStabilization();
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

double Solver::get_norm(double *x, int n){
    double r=0;
    for(int i=0;i<n;i++)
        r+=x[i]*x[i];
    return sqrt(r);
}
double Solver::normalize(double *x, int n){
    double r=get_norm(x,n);
    if(r<1e-6)
        return 0;
    for(int i=0;i<n;i++)
        x[i]/=r;
    return r;
}

double Solver::product(double*a, double *b,int n){
    double r=0;
    for(int i=0;i<n;i++)
        r+=a[i]*b[i];
    return r;
}

void Solver::orth(double *a, double *b, int n){//|a|=1
    double r=product(a,b,n);
    for(int i=0;i<n;i++)
        b[i]-=r*a[i];
    
}
void Solver::SVD(SimLib::ARRAY<2, double> a, SimLib::ARRAY<2, double>& u, SimLib::ARRAY<1, double>& s, SimLib::ARRAY<2, double>& v) {
    int m = a.dim(1);
    int n = a.dim(2);
    int k = n;
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

    double *left_vector=new double[m];
    double *next_left_vector=new double[m];
    double *right_vector=new double[n];
    double *next_right_vector=new double[n];

    for(int col=0;col<k;col++){
        double diff=1;
        double r=-1;
        while(1){
            for(int i=0;i<m;i++)
                left_vector[i]= (float)rand() / RAND_MAX;
            if(normalize(left_vector, m)>1e-6)
                break;
        }
        
        for(int iter=0;diff>=1e-6 && iter<1000;iter++){
            memset(next_left_vector,0,sizeof(double)*m);
            memset(next_right_vector,0,sizeof(double)*n);
            for(int i=0;i<m;i++)
                for(int j=0;j<n;j++)
                    next_right_vector[j]+=left_vector[i]*a(i+1,j+1);
            
            r=normalize(next_right_vector,n);
            if(r<1e-6) break;
            for(int i=0;i<col;i++)
                orth(&v(i+1,1),next_right_vector,n);
            normalize(next_right_vector,n);
            
            for(int i=0;i<m;i++)
                for(int j=0;j<n;j++)
                    next_left_vector[i]+=next_right_vector[j]*a(i+1,j+1);
            r=normalize(next_left_vector,m);
            if(r<1e-6) break;
            for(int i=0;i<col;i++)
                orth(&u(i+1,1),next_left_vector,m);
            normalize(next_left_vector,m);
            diff=0;
            for(int i=0;i<m;i++){
                double d=next_left_vector[i]-left_vector[i];
                diff+=d*d;
            }
            
            memcpy(left_vector,next_left_vector,sizeof(double)*m);
            memcpy(right_vector,next_right_vector,sizeof(double)*n);
        }
        if(r>=1e-6){
            s(col+1)=r;
            memcpy((char *)&u(col+1,1),left_vector,sizeof(double)*m);
            memcpy((char *)&v(col+1,1),right_vector,sizeof(double)*n);
        }else{
            break;
        }
    }
    delete [] next_left_vector;
    delete [] next_right_vector;
    delete [] left_vector;
    delete [] right_vector;
    
}

//min ||Ax - b||
void Solver::OLS(SimLib::ARRAY<2, double>& a, SimLib::ARRAY<1, double>& b, SimLib::ARRAY<1, double>& x) {
    ARRAY<2, double> u, v;
    ARRAY<1, double> s;
    SVD(a, u, s, v);
    int n = a.dim(1);
    int m = a.dim(2);
    ARRAY<1, double> b1(n);
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= n; ++j) {
            b1(i) += u(j,i) * b(j);
        }
    }
    for (int i = 1; i <= m; ++i) {
        if (s(i) == 0)
            b1(i) = 0;
        else
            b1(i) /= s(i);
    }
    x = ARRAY<1, double>(m);
    for (int i = 1; i <= m; ++i) {
        for (int j = 1; j <= m; ++j) {
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