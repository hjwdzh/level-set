 #ifndef GRID_H_
#define GRID_H_

#include "VECTOR.h"
#include "ARRAY.h"
#include "RANGE.h"

namespace SimLib {


template<class TV>
class GRID {
public:
	typedef typename TV::SCALAR T;
    static const int dimension=TV::dimension;
    typedef VECTOR<int,dimension> TV_INT;
    typedef T SCALAR;
    typedef TV VECTOR_T;
    typedef TV_INT INT_VECTOR_T;
    GRID(){}
    GRID(const VECTOR<int,dimension>& n1, RANGE<TV>& r);
    RANGE<VECTOR<int, 3> > Domain_Indices(int ghost_cell = 0);
    TV_INT counts;
    TV_INT Clamped_Index(TV location) const {
        TV ind = (location-dom_min)*one_over_dX;
        return TV_INT(ind(1),ind(2),ind(3)).Get_Max(TV_INT(1,1,1)).Get_Min(counts);
    }
    TV_INT Clamped_Index_End_Minus_One(const TV& location) const{
        TV ind = (location-dom_min)*one_over_dX;
        return (TV_INT(1,1,1)+TV_INT(ind(1),ind(2),ind(3))).Get_Max(TV_INT(1,1,1)).Get_Min(counts-TV_INT(1,1,1));
    }
    TV X(TV_INT ind) {
        return dom_min + TV(ind(1),ind(2),ind(3)) * dx;
    }
    TV X(int a, int b, int c) {
        return dom_min + TV(a,b,c) * dx;
    }
    bool Include(const TV& p) const {
        return (p <= dom_max && dom_min <= p);
    }
    T Interpolate(ARRAY<3, T>& phi, const TV& p) {
        TV ind = (p-dom_min)*one_over_dX;
        TV_INT ind_int1 = TV_INT(ind(1),ind(2),ind(3));
        T d1 = phi(ind_int1);
        if (d1 > 0)
            return 1e30;
        T d2 = phi(ind_int1 + TV_INT(0,0,1));
        if (d2 > 0)
            return 1e30;
        T d3 = phi(ind_int1 + TV_INT(0,1,0));
        if (d3 > 0)
            return 1e30;
        T d4 = phi(ind_int1 + TV_INT(0,1,1));
        if (d4 > 0)
            return 1e30;
        T d5 = phi(ind_int1 + TV_INT(1,0,0));
        if (d5 > 0)
            return 1e30;
        T d6 = phi(ind_int1 + TV_INT(1,0,1));
        if (d6 > 0)
            return 1e30;
        T d7 = phi(ind_int1 + TV_INT(1,1,0));
        if (d7 > 0)
            return 1e30;
        T d8 = phi(ind_int1 + TV_INT(1,1,1));
        if (d8 > 0)
            return 1e30;
        TV w = ind - TV(ind_int1(1),ind_int1(2),ind_int1(3));
        T dis = 0;
        T f = w(1) * w(2) * w(3);
        dis += d1 * f;
        f = w(1) * w(2) * (1 - w(3));
        dis += d2 * f;
        f = w(1) * (1 - w(2)) * w(3);
        dis += d3 * f;
        f = w(1) * (1 - w(2)) * (1 - w(3));
        dis += d4 * f;
        f = (1 - w(1)) * w(2) * w(3);
        dis += d5 * f;
        f = (1 - w(1)) * w(2) * (1 - w(3));
        dis += d6 * f;
        f = (1 - w(1)) * (1 - w(2)) * w(3);
        dis += d7 * f;
        f = (1 - w(1)) * (1 - w(2)) * (1 - w(3));
        dis += d8 * f;
        return dis;
    }
    TV Gradient(ARRAY<3, T>& phi, const TV& p) {
        TV ind = (p-dom_min)*one_over_dX;
        TV_INT ind_int1 = TV_INT(ind(1) + 0.5,ind(2) + 0.5,ind(3) + 0.5);
        T d1, d2, d3, d4;
        d1 = phi(ind_int1);
        d2 = phi(ind_int1 + TV_INT(0,0,1));
        d3 = phi(ind_int1 + TV_INT(0,1,0));
        d4 = phi(ind_int1 + TV_INT(1,0,0));
        return (TV(d4-d1,d3-d1,d2-d1) * one_over_dX).Normal();
    }
	TV dom_min, dom_max;
	TV dx;
    TV one_over_dX;
    T min_dX;
};
}

#endif