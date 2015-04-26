#include "KDOP.h"
#include <math.h>
#include <algorithm>

using namespace SimLib;
#define fmin(a,b)            (((a) < (b)) ? (a) : (b))
#define fmax(a,b)            (((a) < (b)) ? (b) : (a))
template<int d, class T>
KDOP<d,T>::KDOP()
: min(d), max(d){
}

template<int d, class T>
KDOP<d,T>::KDOP(std::vector<TV>& points)
: min(d), max(d) {
    update(points,0);
}

template<int d, class T>
bool KDOP<d,T>::intersect(const BV<T>* bv) {
    const KDOP<d,T>* kdop = dynamic_cast<const KDOP<d,T>*>(bv);
    if (!kdop)
        return false;
    for (int i = 1; i <= d; ++i) {
        if (min(i) > kdop->max(i) || max(i) < kdop->min(i)) {
            return false;
        }
    }
    return true;
}

template<int d, class T>
void KDOP<d,T>::update(std::vector<TV>& points, Matrix4d* m) {
    assert(d == 3);
    min.Fill(1e30);
    max.Fill(-1e30);
    T value;
    for (int i = 0; i < points.size(); ++i) {
        TV x = points[i];
        if (m) {
            Vector4f v = (*m) * Vector4d(x(1),x(2),x(3),1);
            x = TV(v[0],v[1],v[2]);
        }
        value = x(1);
        min(1) = fmin(min(1),value);
        max(1) = fmax(max(1),value);
        value = x(2);
        min(2) = fmin(min(2),value);
        max(2) = fmax(max(2),value);
        value = x(3);
        min(3) = fmin(min(3), value);
        max(3) = fmax(max(3), value);
    }
}

template<int d, class T>
T KDOP<d,T>::operator()(int x) {
    assert(x <= 3 && x >= 1);
    return (min(x) + max(x)) * 0.5;
}

template<int d, class T>
void KDOP<d,T>::include(BV<T> **start, BV<T> **end) {
    KDOP<3,T>** ptr = (KDOP<3,T>**)start;
    min = (*ptr)->min;
    max = (*ptr)->max;
    for (++ptr; ptr <= (KDOP<3,T>**)end; ++ptr) {
        min.GetMin((*ptr)->min);
        max.GetMax((*ptr)->max);
    }
}

template<int d, class T>
BV<T>* KDOP<d,T>::transform(const Matrix4d& m) {
    KDOP<d,T>* b = new KDOP<d,T>();
    Vector4d center(0.5*(min(1)+max(1)), 0.5*(min(2)+max(2)), 0.5*(min(3)+max(3)), 1);
    Vector4d ncenter = m * center;
    double rx = 0, ry = 0, rz = 0;
    rx = fabs(m.at(0, 0) * (max(1) - center[0])) + fabs(m.at(1,0) * (max(2) - center[1])) + fabs(m.at(2,0) * (max(3) - center[2]));
    ry = fabs(m.at(0, 1) * (max(1) - center[0])) + fabs(m.at(1,1) * (max(2) - center[1])) + fabs(m.at(2,1) * (max(3) - center[2]));
    rz = fabs(m.at(0, 2) * (max(1) - center[0])) + fabs(m.at(1,2) * (max(2) - center[1])) + fabs(m.at(2,2) * (max(3) - center[2]));
    b->min(1) = ncenter.x - rx;
    b->max(1) = ncenter.x + rx;
    b->min(2) = ncenter.y - ry;
    b->max(2) = ncenter.y + ry;
    b->min(3) = ncenter.z - rz;
    b->max(3) = ncenter.z + rz;
    return b;
}

template class SimLib::KDOP<3,float>;