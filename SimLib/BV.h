#ifndef BV_H_
#define BV_H_

#include <vector>
#include "VECTOR.h"
#include "vmath.h"

namespace SimLib {

typedef VECTOR<float,3> TV;
    
template<class T>
class BV {
public:
    virtual bool intersect(const BV*) = 0;
    virtual void update(std::vector<TV>& points, Matrix4d* m) = 0;
};

}
#endif