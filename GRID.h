#ifndef GRID_H_
#define GRID_H_

#include "VECTOR.h"
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
	TV dom_min, dom_max;
	TV dx;
    TV one_over_dX;
    T min_dX;
};
}

#endif