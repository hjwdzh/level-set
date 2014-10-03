#ifndef SEGMENT_H_
#define SEGMENT_H_

#include "VECTOR.h"
namespace SimLib {
template<class T>
    class SEGMENT{
        typedef VECTOR<T,3> TV;
    public:
        SEGMENT(){}
        SEGMENT(const TV& a1, const TV& a2) {
            a = a1;
            b = a2;
        }
        TV a,b;
    };
}

#endif