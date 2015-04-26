#ifndef GJKDETECTOR_H_
#define GJKDETECTOR_H_

#include <vector>
#include "../SimLib/VECTOR.h"
using namespace std;
using namespace SimLib;

namespace GJKDetector{
	bool gjk_overlap(const vector<VECTOR<float,3>> &vertices_a, const vector<VECTOR<float,3>> &vertices_b);
	VECTOR<float,3> poly_min(const vector<VECTOR<float,3>> &vertice
		, const VECTOR<float,3> dir);
}


#endif