#ifndef LEVELSET_H_
#define LEVELSET_H_

#include "GRID.h"
#include "ARRAY.h"
#include <vector>
namespace SimLib {
template<class T_GRID>
class LEVELSET{
public:
    typedef typename T_GRID::SCALAR T;
    typedef typename T_GRID::VECTOR_T TV;
    typedef typename T_GRID::INT_VECTOR_T TV_INT;
    T_GRID& grid;
    ARRAY<3,T>& phi;
    int number_of_ghost_cells;
    LEVELSET(T_GRID& _grid, ARRAY<3,T>& _phi, int number_of_ghost_cells_input = 3)
    : phi(_phi), grid(_grid), number_of_ghost_cells(number_of_ghost_cells_input) {
    }
    void Fast_Marching_Method(T stoping_distance = -1e30);
    std::pair<T, TV> Intersect(const TV& p);
private:
    void Down_Adjust(int i);
    void Up_Adjust(int i);
    TV_INT Extract();
    void Extend_Distance(TV_INT& ind);
    void Estimate_Distance(TV_INT& ind);
    std::vector<TV_INT> heap_index;
    ARRAY<3, int> array_ind;
};
}

#endif