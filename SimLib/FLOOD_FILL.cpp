#include "FLOOD_FILL.h"
#include <list>
using namespace SimLib;

int FLOOD_FILL::Flood_Fill(ARRAY<3,int>& colors,ARRAY<3,int>& edge_is_blocked_x,ARRAY<3,int>& edge_is_blocked_y,ARRAY<3,int>& edge_is_blocked_z) {
    RANGE<TV_INT> range = colors.domain();
    int fill_color = 0;
    if (single_cell) {
        for (int i = range.min(1); i <= range.max(1); ++i)
            for (int j = range.min(2); j <= range.max(2); ++j)
                for (int k = range.min(3); k <= range.max(3); ++k) {
                    if ((i == range.min(1) || edge_is_blocked_x(i,j,k) == 1)
                     && (i == range.max(1) || edge_is_blocked_x(i+1,j,k) == 1)
                     && (j == range.min(2) || edge_is_blocked_y(i,j,k) == 1)
                     && (j == range.max(2) || edge_is_blocked_y(i,j+1,k) == 1)
                     && (k == range.min(3) || edge_is_blocked_z(i,j,k+1) == 1)
                     && (k == range.max(3) || edge_is_blocked_z(i,j,k+1) == 1))
                        colors(i,j,k) = fill_color;
                }
    }
    for (int i = range.min(1); i <= range.max(1); ++i)
        for (int j = range.min(2); j <= range.max(2); ++j)
            for (int k = range.min(3); k <= range.max(3); ++k) {
                if (colors(i,j,k) == 0) {
                    Expand(i,j,k,range,colors,edge_is_blocked_x,edge_is_blocked_y,edge_is_blocked_z,++fill_color);
                }
            }
    return fill_color;
}


void FLOOD_FILL::Expand(int x, int y, int z, RANGE<TV_INT>& r, ARRAY<3,int>& colors,ARRAY<3,int>& edge_is_blocked_x,ARRAY<3,int>& edge_is_blocked_y,ARRAY<3,int>& edge_is_blocked_z, int color) {
    std::list<TV_INT> stack;
    stack.push_back(TV_INT(x,y,z));
    colors(stack.back()) = color;
    while (!stack.empty()) {
        TV_INT& indice = stack.front();
        int x = indice(1), y = indice(2), z = indice(3);
        if (x > r.min(1) && !edge_is_blocked_x(x,y,z) && colors(x - 1, y, z) == 0){
            colors(x - 1, y, z) = color;
            stack.push_back(TV_INT(x - 1, y, z));
        }
        if (x < r.max(1) && !edge_is_blocked_x(x+1,y,z) && colors(x + 1, y, z) == 0){
            colors(x + 1, y, z) = color;
            stack.push_back(TV_INT(x + 1, y, z));
        }
        if (y > r.min(2) && !edge_is_blocked_y(x,y,z) && colors(x, y - 1, z) == 0){
            colors(x, y - 1, z) = color;
            stack.push_back(TV_INT(x, y - 1, z));
        }
        if (y < r.max(2) && !edge_is_blocked_y(x,y+1,z) && colors(x, y + 1, z) == 0){
            colors(x, y + 1, z) = color;
            stack.push_back(TV_INT(x, y + 1, z));
        }
        if (z > r.min(3) && !edge_is_blocked_z(x,y,z) && colors(x, y, z - 1) == 0){
            colors(x, y, z - 1) = color;
            stack.push_back(TV_INT(x, y, z - 1));
        }
        if (z < r.max(3) && !edge_is_blocked_z(x,y,z+1) && colors(x, y, z + 1) == 0){
            colors(x, y, z + 1) = color;
            stack.push_back(TV_INT(x, y, z + 1));
        }
        stack.pop_front();
    }
}


void FLOOD_FILL::Identify_Colors_Touching_Boundary(int number_of_colors, ARRAY<3,int>& colors, std::vector<bool> &color_touches_boundary) {
    RANGE<TV_INT> range = colors.domain();
    color_touches_boundary.resize(number_of_colors, false);
    for (int i = range.min(1); i <= range.max(1); ++i)
        for (int j = range.min(2); j <= range.max(2); ++j) {
            int left = colors(i,j,range.min(3));
            int right = colors(i,j,range.max(3));
            if (left > 0)
                color_touches_boundary[left] = true;
            if (right > 0)
                color_touches_boundary[right] = true;
        }
    for (int i = range.min(1); i <= range.max(1); ++i)
        for (int j = range.min(3); j <= range.max(3); ++j) {
            int left = colors(i,range.min(2),j);
            int right = colors(i,range.max(2),j);
            if (left > 0)
                color_touches_boundary[left] = true;
            if (right > 0)
                color_touches_boundary[right] = true;
        }
    for (int i = range.min(2); i <= range.max(2); ++i)
        for (int j = range.min(3); j <= range.max(3); ++j) {
            int left = colors(range.min(1),i,j);
            int right = colors(range.max(1),i,j);
            if (left > 0)
                color_touches_boundary[left] = true;
            if (right > 0)
                color_touches_boundary[right] = true;
        }
}

