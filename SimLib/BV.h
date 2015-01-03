#ifndef BV_H_
#define BV_H_

#include <vector>
#include "VECTOR.h"
#include "vmath.h"

class Rigid_Geometry;

namespace SimLib {

typedef VECTOR<float,3> TV;

template<class T>
class BV {
public:
    BV()
    : rgd(0)
    {}
    virtual bool intersect(const BV*) = 0;
    virtual void update(std::vector<TV>& points, Matrix4d* m) = 0;
    virtual void include(BV** start, BV** end) = 0;
    virtual T operator()(int x) = 0;
    virtual ~BV() {}
    Rigid_Geometry* rgd;
};

}
#endif