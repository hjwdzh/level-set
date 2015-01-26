#include "KDOP.h"
#include <math.h>
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
    assert(d == 4);
    min.Fill(1e30);
    max.Fill(-1e30);
    T value;
    for (int i = 0; i < points.size(); ++i) {
        TV x = points[i];
        if (m) {
            Vector4f v = (*m) * Vector4d(x(1),x(2),x(3),1);
            x = TV(v[0],v[1],v[2]);
        }
        value = x(1) + x(2) + x(3);
        min(1) = fmin(min(1),value);
        max(1) = fmax(max(1),value);
        value = x(1) + x(2) - x(3);
        min(2) = fmin(min(2),value);
        max(2) = fmax(max(2),value);
        value = x(1) - x(2) + x(3);
        min(3) = fmin(min(3), value);
        max(3) = fmax(max(3), value);
        value = -x(1) + x(2) + x(3);
        min(4) = fmin(min(4), value);
        max(4) = fmax(max(4), value);
    }
}

template<int d, class T>
T KDOP<d,T>::operator()(int x) {
    assert(x <= 4 && x >= 1);
    return (min(x) + max(x)) * 0.5;
}

template<int d, class T>
void KDOP<d,T>::include(BV<T> **start, BV<T> **end) {
    KDOP<4,T>** ptr = (KDOP<4,T>**)start;
    min = (*ptr)->min;
    max = (*ptr)->max;
    for (++ptr; ptr <= (KDOP<4,T>**)end; ++ptr) {
        min.GetMin((*ptr)->min);
        max.GetMax((*ptr)->max);
    }
}

template class SimLib::KDOP<4,float>;