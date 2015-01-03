#ifndef TRIANGLULATED_SURFACE_H_
#define TRIANGLULATED_SURFACE_H_

#include <vector>
#include "RANGE.h"
#include "TRIANGLE.h"
#include "RAY.h"
#include "vmath.h"
#include "BV.h"

namespace SimLib {
template<class T>
class TRIANGULATED_SURFACE{
    typedef VECTOR<T,3> TV;
public:
	TRIANGULATED_SURFACE(){}
	std::vector<TRIANGLE<T> > triangle_list;
	Vector3d Update_Bounding_Box_And_Gravity_Center();
	bool loadOBJ(const char * path);
    bool Inside(const TV& point);
    bool Test_Inside_Using_Ray(RAY<T>& ray);
	RANGE<VECTOR<T, 3> > bounding_box;
    VECTOR<T,3> gravity_center;
    std::vector<VECTOR<T,3> > vertices;
    
};
}

#endif