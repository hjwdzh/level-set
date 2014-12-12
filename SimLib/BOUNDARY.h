#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include "ARRAY.h"
#include "GRID.h"
namespace SimLib {

class BOUNDARY {
    typedef float T;
    typedef VECTOR<T,3> TV;
    typedef VECTOR<int,3> TV_INT;
public:
    static void Fill_Ghost_Cells(GRID<TV>& grid,ARRAY<3,T>& phi,ARRAY<3,T>& phi_ghost);
};

}

#endif