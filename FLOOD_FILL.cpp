#include "FLOOD_FILL.h"

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
                        colors(i,j,k) = ++fill_color;
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
    colors(x,y,z) = color;
    if (x > r.min(1) && edge_is_blocked_x(x,y,z))
        Expand(x - 1, y, z, r, colors, edge_is_blocked_x, edge_is_blocked_y, edge_is_blocked_z, color);
    if (x < r.max(1) && edge_is_blocked_x(x+1,y,z))
        Expand(x + 1, y, z, r, colors, edge_is_blocked_x, edge_is_blocked_y, edge_is_blocked_z, color);
    if (y > r.min(2) && edge_is_blocked_y(x,y,z))
        Expand(x, y - 1, z, r, colors, edge_is_blocked_x, edge_is_blocked_y, edge_is_blocked_z, color);
    if (y < r.max(2) && edge_is_blocked_y(x,y+1,z))
        Expand(x, y + 1, z, r, colors, edge_is_blocked_x, edge_is_blocked_y, edge_is_blocked_z, color);
    if (z > r.min(3) && edge_is_blocked_z(x,y,z))
        Expand(x, y, z - 1, r, colors, edge_is_blocked_x, edge_is_blocked_y, edge_is_blocked_z, color);
    if (z < r.max(3) && edge_is_blocked_z(x,y,z+1))
        Expand(x, y, z + 1, r, colors, edge_is_blocked_x, edge_is_blocked_y, edge_is_blocked_z, color);
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

