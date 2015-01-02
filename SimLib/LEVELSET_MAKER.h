#ifndef LEVELSET_MAKER_H_
#define LEVELSET_MAKER_H_

#include "VECTOR.h"
#include "RANGE.h"
#include <vector>
#include "TRIANGULATED_SURFACE.h"
#include "ARRAY.h"
#include "GRID.h"

namespace SimLib {
template<class T>
class LEVELSET_MAKER{
    typedef VECTOR<T,3> TV;
public:
    LEVELSET_MAKER() {
        use_fmm = true;
        surface_thickness = 0.0;
        padding = 0.0;
        fmm_band = 0;
        boundary_outside = false;
        boundary_inside = true;
        flip_inside = true;
        compute_signed_distance_function = true;
        compute_unsigned_distance_function = false;
        compute_heaviside_function = false;
        phi_offset = 0;
    }
    void Set_Surface_Padding_For_Flood_Fill(T padding);
    void Set_Surface_Thickness(T surface_thickness);
    void Compute_Signed_Distance_Function();
    void Use_Fast_Marching_Method(bool fmm, T fmm_band);
    void Only_Boundary_Region_Is_Outside(bool boundary_outside);
    void Keep_Only_Largest_Inside_Region(bool boundary_inside);
    void Flip_Sign_If_Corners_Are_Inside(bool flip_inside);
    void Set_Phi_Offset(T phi_offset);
    void Compute_Level_Set(TRIANGULATED_SURFACE<T>& triangulated_surface, GRID<TV>& grid, ARRAY<3,T>& phi, ARRAY<3,int>& closest_triangle_index);
    bool use_fmm;
    T surface_thickness;
    T padding;
    T fmm_band;
    bool boundary_outside;
    bool boundary_inside;
    bool flip_inside;
    bool compute_signed_distance_function;
    bool compute_unsigned_distance_function;
    bool compute_heaviside_function;
    int positive_boundary;
    T phi_offset;
    std::vector<VECTOR<int,3> > initialized_indices;
};
}

#endif