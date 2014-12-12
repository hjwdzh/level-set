#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include "VECTOR.h"
#include "RANGE.h"
namespace SimLib {
template<class T>
class TRIANGLE{
public:
    typedef VECTOR<T,3> TV;
	TRIANGLE(){}
	TRIANGLE(const VECTOR<T,3>& _a, const VECTOR<T,3>& _b, const VECTOR<T,3>& _c, const TV& _n1=TV(), const TV& _n2=TV(), const TV& _n3=TV(), const VECTOR<T,2>& _t1=VECTOR<T,2>(), const VECTOR<T,2>& _t2=VECTOR<T,2>(), const VECTOR<T,2>& _t3=VECTOR<T,2>());
	RANGE<VECTOR<T,3> > Bounding_Box();
    void Change_Size(const T delta);
    VECTOR<T,3> Closest_Point(const VECTOR<T,3>& location) const;
    T Area() const;
    TV center() const {
        return (a + b + c) * (1 / 3.0f);
    }
	VECTOR<T,3> a, b, c, n1, n2, n3;
    VECTOR<T,2> t1, t2, t3;
};
}

#endif