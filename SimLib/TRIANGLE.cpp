#include <algorithm>
#include "TRIANGLE.h"

using namespace SimLib;

template<class T>
TRIANGLE<T>::TRIANGLE(const VECTOR<T,3>& _a, const VECTOR<T,3>& _b, const VECTOR<T,3>& _c,
                      const VECTOR<T,3>& _n1, const VECTOR<T,3>& _n2, const VECTOR<T,3>& _n3,
                      const VECTOR<T,2>& _t1, const VECTOR<T,2>& _t2, const VECTOR<T,2>& _t3) {
	a = _a;
	b = _b;
	c = _c;
    n1 = _n1;
    n2 = _n2;
    n3 = _n3;
    t1 = _t1;
    t2 = _t2;
    t3 = _t3;
}

template<class T>
RANGE<VECTOR<T,3> > TRIANGLE<T>::Bounding_Box() {
    RANGE<VECTOR<T,3> > range;
	range.min = VECTOR<T, 3>(1e30, 1e30, 1e30);
	range.max = -range.min;
	for (int i = 1; i <= 3; ++i) {
		range.min(i) = std::min(range.min(i), a(i));
		range.max(i) = std::max(range.max(i), a(i));
		range.min(i) = std::min(range.min(i), b(i));
		range.max(i) = std::max(range.max(i), b(i));
		range.min(i) = std::min(range.min(i), c(i));
		range.max(i) = std::max(range.max(i), c(i));
	}
	return range;
}

template<class T>
void TRIANGLE<T>::Change_Size(const T delta) {
    VECTOR<T,3> edge_lengths((c-b).Magnitude(),(a-c).Magnitude(),(b-a).Magnitude());
    T perimeter=edge_lengths.Sum(),area=Area();
    if(!perimeter || !area) return; // don't know which direction to enlarge a degenerate triangle, so do nothing
    T scale=1+delta*(T).5*perimeter/area;
    VECTOR<T,3> weights = edge_lengths / perimeter;
    VECTOR<T,3> incenter= weights(1) * a + weights(2) * b + weights(3) * c;
    a=incenter+(a-incenter)*scale;
    b=incenter+(b-incenter)*scale;
    c=incenter+(c-incenter)*scale;
}

template<class T>
T TRIANGLE<T>::Area() const {
    return (T).5*VECTOR<T,3>::Cross_Product(b-a,c-a).Magnitude();
}

template<class T>
VECTOR<T,3> TRIANGLE<T>::Closest_Point(const VECTOR<T,3>& location) const {
    typedef VECTOR<T,3> TV;
    TV D = a - location;
    TV E0 = b - a;
    TV E1 = c - a;
    T _a = E0.Dot_Product(E0);
    T _b = E0.Dot_Product(E1);
    T _c = E1.Dot_Product(E1);
    T _d = E0.Dot_Product(D);
    T _e = E1.Dot_Product(D);
    T det = _a * _c - _b * _b;
    T s = _b * _e - _c * _d;
    T t = _b * _d - _a * _e;
    if (s + t <= det && fabs(det) > 1e-4) {
        if (s < 0) {
            if (t < 0) {
                s = 0;
                t = 0;
            } else {
                s = 0;
                t = (_e >= 0? 0 : (-_e >=_c ? 1 : -_e/_c));
            }
        } else {
            if (t < 0) {
                t = 0;
                s = (_d >= 0? 0 : (-_d >= _a ? 1 : -_d/_a));
            } else {
                T invDet = 1 / det;
                s *= invDet;
                t *= invDet;
            }
        }
    } else {
        if (s < 0) {
            s = 0;
            t = 1;
        } else {
            if (t < 0) {
                s = 1;
                t = 0;
            } else {
                s = (_c + _e - _b - _d) / (_a - 2 * _b + _c);
                if (s > 1) {
                    s = 1;
                }
                if (s < 0) {
                    s = 0;
                }
                t = 1 - s;
            }
        }
    }
    return a + s * E0 + t * E1;
}

template class SimLib::TRIANGLE<float>;