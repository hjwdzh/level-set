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
/*    bool TransformInclude(RANGE& r, const Matrix4d& m) {
        Vector4d a1(min(1),min(2),min(3),1);
        Vector4d a2(min(1),min(2),max(3),1);
        Vector4d a3(min(1),max(2),min(3),1);
        Vector4d a4(min(1),max(2),max(3),1);
        Vector4d a5(max(1),min(2),min(3),1);
        Vector4d a6(max(1),min(2),max(3),1);
        Vector4d a7(max(1),max(2),min(3),1);
        Vector4d a8(max(1),max(2),max(3),1);
        a1 = m * a1, a2 = m * a2, a3 = m * a3, a4 = m * a4,
        a5 = m * a5, a6 = m * a6, a7 = m * a7, a8 = m * a8;
        return r.Contain(TV(a1[0],a1[1],a1[2])) ||
        r.Contain(TV(a2[0],a2[1],a2[2])) ||
        r.Contain(TV(a3[0],a3[1],a3[2])) ||
        r.Contain(TV(a4[0],a4[1],a4[2])) ||
        r.Contain(TV(a5[0],a5[1],a5[2])) ||
        r.Contain(TV(a6[0],a6[1],a6[2])) ||
        r.Contain(TV(a7[0],a7[1],a7[2])) ||
		r.Contain(TV(a8[0],a8[1],a8[2]));
	}
*/	double getMin(int d) {
		assert(d >= 1 && d <= 3);
		return min.getDim(d);
	}
	double getMax(int d) {
		assert(d >= 1 && d <= 3);
		return max.getDim(d);
	}
	TV min, max;
};
}

#endif