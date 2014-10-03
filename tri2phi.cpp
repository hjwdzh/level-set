#include <iostream>
#include "TRIANGULATED_SURFACE.h"
#include "GRID.h"
#include "LEVELSET_MAKER.h"

using namespace SimLib;

typedef float T;
typedef VECTOR<T, 3> TV;

GRID<TV> Make_Cube_Grid(VECTOR<int,3>& grid_size,const int boundary_cells, RANGE<VECTOR<T,3> >& box){
	GRID<TV> grid=GRID<VECTOR<T,3> >(grid_size,box);
    // Want to make voxels into cubes
    T voxel_size=grid.min_dX;
    int number_of_cells_m,number_of_cells_n,number_of_cells_mn;
    
    TV box_size=box.Edge_Lengths();
    TV box_center=box.Center();
    
    number_of_cells_m=(int)(box.X()/voxel_size+(T).5)+2*boundary_cells;
    number_of_cells_n=(int)(box.Y()/voxel_size+(T).5)+2*boundary_cells;
    number_of_cells_mn=(int)(box.Z()/voxel_size+(T).5)+2*boundary_cells;
    
    TV new_size=voxel_size*TV((T)number_of_cells_m,(T)number_of_cells_n,(T)number_of_cells_mn);
    
    RANGE<TV> new_box;
    new_box.min = (box_center-(T).5*new_size);
    new_box.max = (box_center+(T).5*new_size);
    
    return GRID<VECTOR<T,3> >(VECTOR<int,3>(number_of_cells_m+1,number_of_cells_n+1,number_of_cells_mn+1),new_box);
}

template<class T>
void tri2phi(TRIANGULATED_SURFACE<T>& surface) {
	int boundary_cells = 3;
	VECTOR<int,3> grid_size(100, 100, 100);
	surface.Update_Bounding_Box();
	RANGE<TV> box = surface.bounding_box;
	GRID<TV> grid = Make_Cube_Grid(grid_size, boundary_cells, box);
    LEVELSET_MAKER<T> levelset_maker;
    levelset_maker.Set_Surface_Padding_For_Flood_Fill((T)0);
    levelset_maker.Set_Surface_Thickness((T)1e-5);
    levelset_maker.Compute_Signed_Distance_Function();
    levelset_maker.Use_Fast_Marching_Method(true,(T)0);

    levelset_maker.Only_Boundary_Region_Is_Outside(false);
    levelset_maker.Keep_Only_Largest_Inside_Region(false);
    levelset_maker.Flip_Sign_If_Corners_Are_Inside(false);
    levelset_maker.Set_Phi_Offset((T)0);
    ARRAY<3, T> phi(grid.Domain_Indices());
    levelset_maker.Compute_Level_Set(surface,grid,phi);
//    LEVELSET_IMPLICIT_OBJECT<TV> levelset_implicit_surface(grid,phi);
    //phi+=(T)1*grid.Maximum_Edge_Length();
}

int main() {
	//Load Object from file
	TRIANGULATED_SURFACE<T> mesh;
	char file[100];
	std::cin >> file;
	mesh.loadOBJ(file);
	tri2phi(mesh);
	return 0;
}