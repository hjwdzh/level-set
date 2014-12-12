#ifndef IMPLICIT_OBJECT_H_
#define IMPLICIT_OBJECT_H_

#include "VECTOR.h"

namespace SimLib {

template<class T>
class IMPLICIT_OBJECT{
public:
    typedef VECTOR<T,3> TV;
    virtual std::pair<T, TV> Intersect(const TV& p) = 0;
};
}
#endif