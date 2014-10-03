#include "LEVELSET.h"

using namespace SimLib;

template<class T_GRID>
void LEVELSET<T_GRID>::Fast_Marching_Method(const T time,const T stopping_distance,const std::vector<VECTOR<int,3> >* seed_indices,const bool add_seed_indices_for_ghost_cells)
{}

template class SimLib::LEVELSET<GRID<VECTOR<float,3> > >;