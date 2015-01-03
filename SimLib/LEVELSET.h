#ifndef LEVELSET_H_
#define LEVELSET_H_

#include "GRID.h"
#include "ARRAY.h"
#include "IMPLICIT_OBJECT.h"
#include "TRIANGULATED_SURFACE.h"
#include <vector>
namespace SimLib {
template<class T_GRID>
    class LEVELSET : public IMPLICIT_OBJECT<typename T_GRID::SCALAR>{
public:
    typedef typename T_GRID::SCALAR T;
    typedef typename T_GRID::VECTOR_T TV;
    typedef typename T_GRID::INT_VECTOR_T TV_INT;
    T_GRID& grid;
    ARRAY<3,T>& phi;
    int number_of_ghost_cells;
    LEVELSET(T_GRID& _grid, ARRAY<3,T>& _phi, int number_of_ghost_cells_input = 3)
    : grid(_grid), phi(_phi), number_of_ghost_cells(number_of_ghost_cells_input) {
    }
    void Fast_Marching_Method(TRIANGULATED_SURFACE<float>& tris, ARRAY<3,int>& closest_index, T stoping_distance = -1e30);
    std::pair<T, TV> Intersect(const TV& p);
private:
    void Down_Adjust(int i);
    void Up_Adjust(int i);
    TV_INT Extract();
    void Extend_Distance(TV_INT& ind);
    void Estimate_Distance(TV_INT& ind, TV_INT& parent_ind);
    std::vector<TV_INT> heap_index;
    ARRAY<3, int> array_ind, *closestIndex;
    TRIANGULATED_SURFACE<float>* triList;
};
}

#endif