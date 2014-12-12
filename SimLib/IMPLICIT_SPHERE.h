#ifndef IMPLICIT_SPHERE_H_
#define IMPLICIT_SPHERE_H_

#include "VECTOR.h"
#include "RANGE.h"
#include "IMPLICIT_OBJECT.h"

namespace SimLib {
    
template<class T>
class IMPLICIT_SPHERE: public IMPLICIT_OBJECT<T>{
public:
    typedef VECTOR<T,3> TV;
    IMPLICIT_SPHERE(){}
    IMPLICIT_SPHERE(const TV& _o, T _r) {
        o = _o;
        r = _r;
    }
    std::pair<T, TV> Intersect(const TV& p) {
        TV d = p - o;
        float t = d.Magnitude();
        if (t > r) {
            return make_pair(1e30, TV());
        }
        return make_pair(t - r, d * (1.0f / t));
    }
    TV o;
    T r;
};
}

#endif