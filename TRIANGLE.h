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
	TRIANGLE(const VECTOR<T,3>& _a, const VECTOR<T,3>& _b, const VECTOR<T,3>& _c);
	RANGE<VECTOR<T,3> > Bounding_Box();
    void Change_Size(const T delta);
    VECTOR<T,3> Closest_Point(const VECTOR<T,3>& location,VECTOR<T,3>& weights) const;
    T Area() const;
    TV Barycentric_Coordinates(const TV& location) const {
        TV u=b-a,v=c-a,w=location-a;
        T u_dot_u=u.Dot_Product(u),
          v_dot_v=v.Dot_Product(v),
          u_dot_v=u.Dot_Product(v),
          u_dot_w=u.Dot_Product(w),
          v_dot_w=v.Dot_Product(w);
        T denominator=u_dot_u*v_dot_v-u_dot_v*u_dot_v,
          one_over_denominator;
        if(abs(denominator)>(T)1e-16)
            one_over_denominator=1/denominator;
        else
            one_over_denominator=(T)1e16;
        T a=(v_dot_v*u_dot_w-u_dot_v*v_dot_w)*one_over_denominator,
          b=(u_dot_u*v_dot_w-u_dot_v*u_dot_w)*one_over_denominator;
        return TV(1-a-b,a,b);
    }
	VECTOR<T,3> a, b, c;
};
}

#endif