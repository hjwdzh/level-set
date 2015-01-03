#ifndef KDOP_H_
#define KDOP_H_

#include "BV.h"
#include "VECTOR.h"
#include "ARRAY.h"
#include <vector>

namespace SimLib {
    
template<int d, class T>
class KDOP : public BV<T>{
public:
    typedef VECTOR<T,3> TV;
    KDOP();
    KDOP(std::vector<TV>& points);
    void update(std::vector<TV>& points, Matrix4d* m);
    bool intersect(const BV<T>*);
    void include(BV<T>** start, BV<T>** end);
    T operator()(int x);
    ARRAY<1,T> min;
    ARRAY<1,T> max;
};

}
#endif