#include "GRID.h"

using namespace SimLib;

template<class TV>
GRID<TV>::GRID(const VECTOR<int,dimension>& n1, RANGE<TV>& r) {
    counts = n1;
    dom_min = r.min;
    dom_max = r.max;
    dx = TV((r.max - r.min) / TV(n1(1),n1(2),n1(3)));
    one_over_dX = TV(1,1,1) / dx;
    min_dX = dx.Min();
}

template<class TV>
RANGE<VECTOR<int,3> > GRID<TV>::Domain_Indices(int ghost_cell) {
    return RANGE<VECTOR<int,dimension> >(VECTOR<int, dimension>(1-ghost_cell,1-ghost_cell,1-ghost_cell), counts + VECTOR<int, dimension>(ghost_cell, ghost_cell, ghost_cell));
}


template class SimLib::GRID<VECTOR<float, 3> >;