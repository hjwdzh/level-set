#include <algorithm>
#include "TRIANGLE.h"

using namespace SimLib;

template<class T>
TRIANGLE<T>::TRIANGLE(const VECTOR<T,3>& _a, const VECTOR<T,3>& _b, const VECTOR<T,3>& _c) {
	a = _a;
	b = _b;
	c = _c;
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
VECTOR<T,3> TRIANGLE<T>::Closest_Point(const VECTOR<T,3>& location,VECTOR<T,3>& weights) const {
    weights=Barycentric_Coordinates(location);
    // project closest point to the triangle if it's not already inside it
    if(weights(1)<0){
        VECTOR<T,3> v=c-b;
        T denominator=v.Dot_Product(v);
        T a23 = (denominator == 0) ? 0 : (location-b).Dot_Product(v)/denominator;
        if(a23<0){
            if(weights(3)<0){ // Closest point is on edge x1--x2
                VECTOR<T,3> v = b - a;
                T denominator = v.Dot_Product(v);
                T a12 = (denominator == 0) ? 0 : (location-a).Dot_Product(v)/denominator;
                if (a12 < 0)
                    a12 = 0;
                if (a12 > 1)
                    a12 = 1;
                weights=VECTOR<T,3>(1-a12,a12,0);
                return weights(1)*a+weights(2)*b;
            }
            else{
                weights=VECTOR<T,3>(0,1,0);
                return b;
            }
        } // Closest point is x2
        else if(a23>1){
            if(weights(2)<0){ // Closest point is on edge x1--x3
                VECTOR<T,3> v = c - a;
                T denominator = v.Dot_Product(v);
                T a13 = (denominator == 0) ? 0 : (location - a).Dot_Product(v)/denominator;
                if (a13 < 0)
                    a13 = 0;
                if (a13 > 1)
                    a13 = 1;
                weights=VECTOR<T,3>(1-a13,0,a13);
                return weights(1)*a+weights(3)*c;
            }
            else{
                weights=VECTOR<T,3>(0,0,1);
                return c;
            }
        } // Closest point is x3
        else{
            weights=VECTOR<T,3>(0,1-a23,a23);
            return weights(2)*b+weights(3)*c;
        }
    } // Closest point is on edge x2--x3
    else if(weights(2)<0){
        VECTOR<T, 3> v = c - a;
        T denominator = v.Dot_Product(v);
        T a13 = (denominator == 0) ? 0 : (location-a).Dot_Product(v)/denominator;
        if(a13<0){
            if(weights(3)<0){ // Closest point is on edge x1--x2
                VECTOR<T,3> v = b - a;
                T denominator = v.Dot_Product(v);
                T a12 = (denominator == 0) ? 0 : (location-a).Dot_Product(v)/denominator;
                if (a12 < 0)
                    a12 = 0;
                if (a12 > 1)
                    a12 = 1;
                weights=VECTOR<T,3>(1-a12,a12,0);
                return weights(1)*a+weights(2)*b;
            }
            else{
                weights=VECTOR<T,3>(1,0,0);
                return a;
            }
        } // Closest point is x1
        else if(a13>1){
            weights=VECTOR<T,3>(0,0,1);
            return c;
        } // Closest point is x3
        else{
            weights=VECTOR<T,3>(1-a13,0,a13);
            return weights(1)*a+weights(3)*c;
        }
    } // Closest point is on edge x1--x3
    else if(weights(3)<0){ // Closest point is on edge x1--x2
        VECTOR<T,3> v = b - a;
        T denominator = v.Dot_Product(v);
        T a12 = (denominator == 0) ? 0 : (location-a).Dot_Product(v)/denominator;
        if (a12 < 0)
            a12 = 0;
        if (a12 > 1)
            a12 = 1;
        weights=VECTOR<T,3>(1-a12,a12,0);
        return weights(1)*a+weights(2)*b;
    }
    return weights(1)*a+weights(2)*b+weights(3)*c; // Point is interior to the triangle
}

template class SimLib::TRIANGLE<float>;