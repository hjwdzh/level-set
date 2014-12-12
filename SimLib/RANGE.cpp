#include <algorithm>
#include "RANGE.h"

using namespace SimLib;

template<class TV>
void RANGE<TV>::include(const RANGE<TV>& a) {
	for (int i = 1; i <= 3; ++i) {
		min(i) = std::min(min(i), a.min(i));
		max(i) = std::max(max(i), a.max(i));
	}
}

template<class TV>
void RANGE<TV>::Change_Size(T delta) {
    min -= TV(delta, delta, delta);
    max += TV(delta, delta, delta);
}

template class SimLib::RANGE<VECTOR<float, 3> >;
template class SimLib::RANGE<VECTOR<int, 3> >;