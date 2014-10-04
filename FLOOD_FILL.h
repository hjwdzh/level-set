#ifndef FLOODFILL_H_
#define FLOODFILL_H_

#include "ARRAY.h"
namespace SimLib {
class FLOOD_FILL{
    typedef VECTOR<int,3> TV_INT;
public:
    FLOOD_FILL() {
        single_cell = true;
    }
    
    int Flood_Fill(ARRAY<3,int>& colors,ARRAY<3,int>& edge_is_blocked_x,ARRAY<3,int>& edge_is_blocked_y,ARRAY<3,int>& edge_is_blocked_z);
    
    void Expand(int x, int y, int z, RANGE<TV_INT>& r, ARRAY<3,int>& colors,ARRAY<3,int>& edge_is_blocked_x,ARRAY<3,int>& edge_is_blocked_y,ARRAY<3,int>& edge_is_blocked_z, int color);
    
    void Identify_Colors_Touching_Boundary(int number_of_colors, ARRAY<3,int>& colors, std::vector<bool> &color_touches_boundary);
    
    bool single_cell;
};
}

#endif