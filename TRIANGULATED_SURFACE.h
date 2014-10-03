#ifndef TRIANGLULATED_SURFACE_H_
#define TRIANGLULATED_SURFACE_H_

#include <vector>
#include "RANGE.h"
#include "TRIANGLE.h"

namespace SimLib {
template<class T>
class TRIANGULATED_SURFACE{
public:
	TRIANGULATED_SURFACE(){}
	std::vector<TRIANGLE<T> > triangle_list;
	void Update_Bounding_Box();
	bool loadOBJ(char * path);
	RANGE<VECTOR<T, 3> > bounding_box;
};
}

#endif