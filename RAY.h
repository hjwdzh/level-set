#ifndef RAY_H_
#define RAY_H_

#include "VECTOR.h"
namespace SimLib {
template<class T>
    class RAY{
        typedef VECTOR<T,3> TV;
    public:
        RAY(){
            d = TV(1,0,0);
        }
        RAY(const TV& a1, const TV& a2, bool infinite = false) {
            o = a1;
            d = a2 / a2.Magnitude();
            semi_infinite = infinite;
        }
        TV o,d;
        bool semi_infinite;
        T t_max;
        T intersect;
    };
}

#endif