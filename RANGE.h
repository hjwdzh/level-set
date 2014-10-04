#ifndef RANGE_H_
#define RANGE_H_

#include "VECTOR.h"
namespace SimLib {
template<class TV>
class RANGE{
    typedef typename TV::SCALAR T;
public:
	RANGE(){}
	RANGE(const TV& a, const TV& b) {
		min = a;
		max = b;
	}
    T X() {
        return max(1) - min(1);
    }
    T Y() {
        return max(2) - min(2);
    }
    T Z() {
        return max(3) - min(3);
    }
    TV Edge_Lengths() {
        return max - min;
    }
    TV Center() {
        return (T)0.5 * (max + min);
    }
    void Change_Size(T delta);
	void include(const RANGE& a);
    bool Contain(const TV& v) {
        return (v >= min) && (v <= max);
    }
    bool Lazy_Intersection(RANGE& r) const {
        return min <= r.max && r.min <= max;
    }
	TV min, max;
};
}

#endif