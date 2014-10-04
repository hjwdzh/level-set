#include "LEVELSET_MAKER.h"
#include "INTERSECTION.h"
#include "SEGMENT.h"
#include "LEVELSET.h"
#include "FLOOD_FILL.h"

using namespace SimLib;

template<class T>
void LEVELSET_MAKER<T>::Set_Surface_Padding_For_Flood_Fill(T padding) {
    this->padding = padding;
}

template<class T>
void LEVELSET_MAKER<T>::Set_Surface_Thickness(T surface_thickness) {
    this->surface_thickness = surface_thickness;
}

template<class T>
void LEVELSET_MAKER<T>::Compute_Signed_Distance_Function() {
    compute_signed_distance_function = true;
    compute_unsigned_distance_function = false;
    compute_heaviside_function = false;
}

template<class T>
void LEVELSET_MAKER<T>::Use_Fast_Marching_Method(bool fmm, T fmm_band) {
    use_fmm = fmm;
    fmm_band = fmm_band;
}

template<class T>
void LEVELSET_MAKER<T>::Only_Boundary_Region_Is_Outside(bool boundary_outside) {
    this->boundary_outside = boundary_outside;
}

template<class T>
void LEVELSET_MAKER<T>::Keep_Only_Largest_Inside_Region(bool boundary_inside) {
    this->boundary_inside = boundary_inside;
}

template<class T>
void LEVELSET_MAKER<T>::Flip_Sign_If_Corners_Are_Inside(bool flip_inside) {
    this->flip_inside = flip_inside;
}

template<class T>
void LEVELSET_MAKER<T>::Set_Phi_Offset(T phi_offset) {
    this->phi_offset = phi_offset;
}

template<class T>
void LEVELSET_MAKER<T>::Compute_Level_Set(TRIANGULATED_SURFACE<T>& triangulated_surface, GRID<TV>& grid, ARRAY<3,T>& phi) {
    typedef VECTOR<int, 3> TV_INT;
    T fmm_stopping_distance=fmm_band*grid.dx.Max();
    bool need_flood_fill=compute_signed_distance_function || compute_heaviside_function;
    ARRAY<3,int> edge_is_blocked_x,edge_is_blocked_y,edge_is_blocked_z;
    if(need_flood_fill){
        edge_is_blocked_x.Resize(RANGE<TV_INT>(TV_INT(2,1,1),grid.counts));
        edge_is_blocked_y.Resize(RANGE<TV_INT>(TV_INT(1,2,1),grid.counts));
        edge_is_blocked_z.Resize(RANGE<TV_INT>(TV_INT(1,1,2),grid.counts));
    }
    
    bool store_closest_triangle_index=need_flood_fill && !boundary_outside;
    ARRAY<3,int> closest_triangle_index;
    if(store_closest_triangle_index)
        closest_triangle_index.Resize(grid.Domain_Indices());
    
    bool store_initialized_indices=use_fmm;
    if(store_initialized_indices){
        initialized_indices.resize(0);
        initialized_indices.reserve(20);
    }
    
    const T surface_thickness_over_two=(T).5*(surface_thickness>0?surface_thickness:grid.min_dX/100);
    const T surface_padding_for_flood_fill=padding;
    
    const RANGE<TV>& grid_domain=RANGE<TV>(grid.dom_min, grid.dom_max);
    for(int t=1;t<=triangulated_surface.triangle_list.size();t++){
        const TRIANGLE<T>& triangle=triangulated_surface.triangle_list[t-1];
        TRIANGLE<T> enlarged_triangle=triangle;
        if(surface_padding_for_flood_fill)
            enlarged_triangle.Change_Size(surface_padding_for_flood_fill);
        RANGE<TV> triangle_bounding_box=enlarged_triangle.Bounding_Box();
        triangle_bounding_box.Change_Size(surface_thickness_over_two);
        if(!grid_domain.Lazy_Intersection(triangle_bounding_box))
            continue;
        TV_INT min_index=grid.Clamped_Index(triangle_bounding_box.min),
               max_index=grid.Clamped_Index_End_Minus_One(triangle_bounding_box.max)+TV_INT(1,1,1);
        for(int i=min_index(1);i<=max_index(1);i++)
            for(int j=min_index(2);j<=max_index(2);j++)
                for(int k=min_index(3);k<=max_index(3);k++){
                    TV grid_position=grid.X(i,j,k),
                       weights,
                       closest_point=triangle.Closest_Point(grid_position,weights);
                    T distance_squared=(grid_position-closest_point).Magnitude_Squared();
                    if(phi(i,j,k)>1e20 || distance_squared<phi(i,j,k)*phi(i,j,k)){
                        if(store_initialized_indices && phi(i,j,k)>1e20)
                            initialized_indices.push_back(TV_INT(i,j,k));
                        phi(i,j,k)=sqrt(distance_squared);
                        if(store_closest_triangle_index)
                            closest_triangle_index(i,j,k)=t;
                    }
                }
        if(need_flood_fill){
            for(int i=min_index(1)+1;i<=max_index(1);i++)
                for(int j=min_index(2);j<=max_index(2);j++)
                    for(int k=min_index(3);k<=max_index(3);k++)
                        if(!edge_is_blocked_x(i,j,k))
                            edge_is_blocked_x(i,j,k)=INTERSECTION<T>::Intersects(SEGMENT<T>(grid.X(i,j,k),grid.X(i-1,j,k)),enlarged_triangle,surface_thickness_over_two);
            for(int i=min_index(1);i<=max_index(1);i++)
                for(int j=min_index(2)+1;j<=max_index(2);j++)
                    for(int k=min_index(3);k<=max_index(3);k++)
                        if(!edge_is_blocked_y(i,j,k))
                            edge_is_blocked_y(i,j,k)=INTERSECTION<T>::Intersects(SEGMENT<T>(grid.X(i,j,k),grid.X(i,j-1,k)),enlarged_triangle,surface_thickness_over_two);
            for(int i=min_index(1);i<=max_index(1);i++)
                for(int j=min_index(2);j<=max_index(2);j++)
                    for(int k=min_index(3)+1;k<=max_index(3);k++)
                        if(!edge_is_blocked_z(i,j,k))
                            edge_is_blocked_z(i,j,k)=INTERSECTION<T>::Intersects(SEGMENT<T>(grid.X(i,j,k),grid.X(i,j,k-1)),enlarged_triangle,surface_thickness_over_two);
        }
    }
    
    if((compute_signed_distance_function || compute_unsigned_distance_function) && use_fmm && fmm_stopping_distance) {
        for(int i=1;i<=grid.counts(1);i++)
            for(int j=1;j<=grid.counts(2);j++)
                for(int k=1;k<=grid.counts(3);k++)
                    phi(i,j,k)=std::min(phi(i,j,k),fmm_stopping_distance);
    }
    else if(compute_heaviside_function) {
        phi.Fill(grid.dx.Max());
    }
    
    if(need_flood_fill){ // Need flood fill to determine sign (inside/outside)
        ARRAY<3,int> colors(grid.Domain_Indices());
        FLOOD_FILL flood_fill;
        int number_of_colors=flood_fill.Flood_Fill(colors,edge_is_blocked_x,edge_is_blocked_y,edge_is_blocked_z);
        if(number_of_colors==1 && !phi_offset){ // there is only one color. check if the whole domain is inside or outside then return
            if(triangulated_surface.bounding_box.Contain(grid.X(1,1,1)))
                phi.Fill(-1e30);
            else
                phi.Fill(1e30);
            return;
        }
        std::vector<bool> color_is_inside(number_of_colors);
        if(boundary_outside){
            std::vector<bool> color_touches_boundary(number_of_colors);
            flood_fill.Identify_Colors_Touching_Boundary(number_of_colors,colors,color_touches_boundary);
            for(int i=0;i<number_of_colors;i++)
                color_is_inside[i]=!color_touches_boundary[i];
        }
        else{
            std::vector<T> color_maximum_distance(number_of_colors,-1);
            std::vector<TV_INT> color_representatives(number_of_colors);
            for(int i=1;i<=grid.counts(1);i++)
                for(int j=1;j<=grid.counts(2);j++)
                    for(int k=1;k<=grid.counts(3);k++)
                        if(closest_triangle_index(i,j,k) && color_maximum_distance[colors(i,j,k)-1]<phi(i,j,k)){
                            color_maximum_distance[colors(i,j,k)-1]=phi(i,j,k);
                            color_representatives[colors(i,j,k)-1]=TV_INT(i,j,k);
                        }
            for(int color=1;color<=number_of_colors;color++){
                if(color_maximum_distance[color-1]<0){
                    phi.Fill(0);
                    return;
                }
                else {
                    color_is_inside[color-1]=triangulated_surface.Inside(grid.X(color_representatives[color-1]));
                }
            }
        }
        if(boundary_inside){
            std::vector<int> region_size(number_of_colors, 0);
            for(int i=1;i<=grid.counts(1);i++)
                for(int j=1;j<=grid.counts(2);j++)
                    for(int k=1;k<=grid.counts(3);k++)
                        if(colors(i,j,k)>0 && color_is_inside[colors(i,j,k)-1])
                            region_size[colors(i,j,k)-1]++;
            int max_region_size=region_size[0];
            for (int i = 1; i < number_of_colors; ++i)
                max_region_size = std::max(max_region_size, region_size[i]);
            for(int i=1;i<=number_of_colors;i++)
                if(color_is_inside[i-1] && region_size[i-1]<max_region_size)
                    color_is_inside[i-1]=false;
        }
        if(flip_inside){ // If the majority of corners are labelled as inside then we flip signs
            int num_corners_inside=(int)color_is_inside[colors(1,1,1)-1]
                                  +(int)color_is_inside[colors(1,1,grid.counts(3))-1]
                                  +(int)color_is_inside[colors(1,grid.counts(2),1)-1]
                                  +(int)color_is_inside[colors(1,grid.counts(2),grid.counts(3)-1)]
                                  +(int)color_is_inside[colors(grid.counts(1),1,1)-1]
                                  +(int)color_is_inside[colors(grid.counts(1),1,grid.counts(2))-1]
                                  +(int)color_is_inside[colors(grid.counts(1),grid.counts(2),1)-1]
                                  +(int)color_is_inside[colors(grid.counts(1),grid.counts(2),grid.counts(3))-1];
            if(num_corners_inside>4){
                for(int i=1;i<=number_of_colors;i++)
                    color_is_inside[i-1]=!color_is_inside[i-1];
            }
        }
        for(int i=1;i<=grid.counts(1);i++)
            for(int j=1;j<=grid.counts(2);j++)
                for(int k=1;k<=grid.counts(3);k++)
                    if(color_is_inside[colors(i,j,k)-1])
                        phi(i,j,k)*=-1;
    }
    
    if(use_fmm && (compute_unsigned_distance_function || compute_signed_distance_function)){
        for(int i=1;i<=grid.counts(1);i++)
            for(int j=1;j<=grid.counts(2);j++)
                for(int k=1;k<=grid.counts(3);k++)
                    phi(i,j,k)=std::min(std::max(phi(i,j,k),-10*grid.min_dX),10*grid.min_dX);
        GRID<TV> grid_copy=grid;
        LEVELSET<GRID<TV> > levelset(grid_copy,phi);
        if(compute_unsigned_distance_function)
            levelset.Fast_Marching_Method(0,fmm_stopping_distance,&initialized_indices);
        else if(compute_signed_distance_function)
            levelset.Fast_Marching_Method(0,fmm_stopping_distance,phi_offset?&initialized_indices:0);
    }
    
    if(phi_offset){
        phi-=phi_offset;
        if(use_fmm && compute_signed_distance_function)
            LEVELSET<GRID<TV> >(grid,phi).Fast_Marching_Method(0,fmm_stopping_distance);
    }
    
    return;
}

template class SimLib::LEVELSET_MAKER<float>;