#ifndef LEVELSET_H_
#define LEVELSET_H_

#include "GRID.h"
#include "ARRAY.h"
namespace SimLib {
template<class T_GRID>
class LEVELSET{
    typedef typename T_GRID::SCALAR T;
    typedef typename T_GRID::VECTOR_T TV;
    typedef typename T_GRID::INT_VECTOR_T TV_INT;
public:
    T_GRID grid;
    ARRAY<3,T> phi;
    LEVELSET(T_GRID& _grid, ARRAY<3,T>& _phi) {
        grid = _grid;
        phi = _phi;
    }
    void Fast_Marching_Method(const T time=0,const T stopping_distance=0,const std::vector<VECTOR<int,3> >* seed_indices=0,const bool add_seed_indices_for_ghost_cells=false);
};
}

#endif