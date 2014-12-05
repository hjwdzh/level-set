#include "BOUNDARY.h"

using namespace SimLib;

void BOUNDARY::Fill_Ghost_Cells(GRID<TV>& grid,ARRAY<3,T>& phi,ARRAY<3,T>& phi_ghost) {
    RANGE<TV_INT> ghost_range = phi_ghost.domain();
    RANGE<TV_INT> range = phi.domain();
    for (int i = ghost_range.min(1); i < range.min(1); ++i)
        for (int j = range.min(2); j <= range.max(2); ++j)
            for (int k = range.min(3); j <= range.max(3); ++k)
                phi_ghost(i,j,k)=phi(range.min(1),j,k);
    for (int i = range.max(1)+1; i <= ghost_range.max(1); ++i)
        for (int j = range.min(2); j <= range.max(2); ++j)
            for (int k = range.min(3); j <= range.max(3); ++k)
                phi_ghost(i,j,k)=phi(range.max(1),j,k);
    for (int i = ghost_range.min(1); i <= ghost_range.max(1); ++i)
        for (int j = ghost_range.min(2); j < range.min(2); ++j)
            for (int k = range.min(3); k <= range.max(3); ++k)
                phi_ghost(i,j,k)=phi(i,range.min(2),k);
    for (int i = ghost_range.min(1); i <= ghost_range.max(1); ++i)
        for (int j = range.max(2)+1; j <= ghost_range.max(2); ++j)
            for (int k = range.min(3); k <= range.max(3); ++k)
                phi_ghost(i,j,k)=phi(i,range.max(2),k);
    for (int i = ghost_range.min(1); i <= ghost_range.max(1); ++i)
        for (int j = ghost_range.min(2); j <= ghost_range.max(2); ++j)
            for (int k = ghost_range.min(3); k < range.min(3); ++k)
                phi_ghost(i,j,k)=phi(i,j,range.min(3));
    for (int i = ghost_range.min(1); i <= ghost_range.max(1); ++i)
        for (int j = ghost_range.min(2); j <= ghost_range.max(2); ++j)
            for (int k = range.max(3)+1; k <= ghost_range.max(3); ++k)
                phi_ghost(i,j,k)=phi(i,j,range.max(3));
}
