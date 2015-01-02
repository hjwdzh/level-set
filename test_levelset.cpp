#include "LEVELSET_MAKER.h"
#include "LEVELSET.h"
#include "string.h"
#include "VECTOR.h"
#include "TRIANGULATED_SURFACE.h"
#include <iostream>
#include "SysBowling.h"
#include "Camera.h"
using namespace std;
using namespace SimLib;

typedef VECTOR<float, 3> TV;
typedef VECTOR<int, 3> TV_INT;

const static int g_WindowPosX = 240;
const static int g_WindowPosY = 200;

int g_WindowWidth = 512;
int g_WindowHeight = 384;
double g_d = 1.0;
SysBowling* g_sys = 0;
Camera* g_camera = 0;
string res_path;
double g_top = 1.0;
double g_right = g_top * (double)g_WindowWidth / g_WindowHeight;


LEVELSET<GRID<TV> >* createTriMeshAndLevelSet(const char* name) {
	string n = string(name);
    TRIANGULATED_SURFACE<float>* triangles = new TRIANGULATED_SURFACE<float>();
    triangles->loadOBJ(name);
    triangles->Update_Bounding_Box_And_Gravity_Center();
    RANGE<TV> range = triangles->bounding_box;
    range.min -= TV(0.2, 0.2, 0.2);
    range.max += TV(0.2, 0.2, 0.2);
    GRID<TV>* grid = new GRID<TV>(TV_INT(40,40,40), range);
    ARRAY<3,float>* phi = new ARRAY<3, float>(grid->Domain_Indices());
    LEVELSET<GRID<TV> >* implicit_object = new LEVELSET<GRID<TV> >(*grid, *phi);
    SimLib::LEVELSET_MAKER<float> level_maker;
    ARRAY<3, int> closest_index;
    level_maker.Compute_Level_Set(*triangles, *grid, *phi, closest_index);
    ((LEVELSET<GRID<TV> >*)implicit_object)->Fast_Marching_Method(*triangles, closest_index);
    return implicit_object;
}

bool test_sphere() {
	LEVELSET<GRID<TV> >* implicit_object = createTriMeshAndLevelSet("/Users/jingweihuang/Desktop/projects/levelset/Resources/models/unit.obj");
    TV_INT indd;
    TV xx;
    for (int i = 1; i <= implicit_object->phi.dim(1); ++i) {
        for (int j = 1; j <= implicit_object->phi.dim(2); ++j) {
            for (int k = 1; k <= implicit_object->phi.dim(3); ++k) {
                TV x = implicit_object->grid.X(TV_INT(i,j,k));
                TV normal = x.Normal();
				pair<float, TV> intersection = implicit_object->Intersect(x);
                if (intersection.first > 0)
                    continue;
                double angle = acos(intersection.second.Dot_Product(normal)) / 3.141592654 * 180;
                if (angle > 15 && x.Magnitude() > 0.2 && x.Magnitude() < 0.8) {
                    return false;
                }
			}
		}
	}
    return true;
}

bool test_cube() {
	LEVELSET<GRID<TV> >* implicit_object = createTriMeshAndLevelSet("/Users/jingweihuang/Desktop/projects/levelset/Resources/models/cube.obj");
    for (int i = 1; i <= implicit_object->phi.dim(1); ++i) {
        for (int j = 1; j <= implicit_object->phi.dim(2); ++j) {
            for (int k = 1; k <= implicit_object->phi.dim(3); ++k) {
				double min_dis = 2;
                TV x = implicit_object->grid.X(TV_INT(i,j,k));
                if (x(1) < -0.95 || x(1) > 0.95 || x(2) < -0.95 || x(2) > 0.95 || x(3) < -0.95 || x(3) > 0.95)
                    continue;
				TV normal = TV(0,0,0);
				if (min_dis > x(1) + 1) {
					min_dis = x(1) + 1;
					normal = TV(-1,0,0);
				}
				if (min_dis > 1 - x(1)) {
					min_dis = 1 - x(1);
					normal = TV(1, 0, 0);
				}
				if (min_dis > x(2) + 1) {
					min_dis = x(2) + 1;
					normal = TV(0, -1, 0);
				}
				if (min_dis > 1 - x(2)) {
					min_dis = 1 - x(2);
					normal = TV(0, 1, 0);
				}
				if (min_dis > x(3) + 1) {
					min_dis = x(3) + 1;
					normal = TV(0, 0, -1);
				}
				if (min_dis > 1 - x(3)) {
					min_dis = 1 - x(3);
					normal = TV(0, 0, 1);
				}
				pair<float, TV> intersection = implicit_object->Intersect(x);
                double angle = acos(normal.Dot_Product(intersection.second)) / 3.141502654 * 180.0;
				if (angle > 5 && fabs(fabs(x(1)) - fabs(x(2))) > 1e-4 && fabs(fabs(x(1)) - fabs(x(3))) > 1e-4 && fabs(fabs(x(2)) - fabs(x(3))) > 1e-4) {
                    return false;
				}
			}
		}
	}
    return true;
}

int main() {
    if (test_cube()) {
        cout << "Test cube successfully!\n";
    } else {
        cout << "Test cube wrong!\n";
    }
    if (test_sphere()) {
        cout << "Test sphere successfully!\n";
    } else {
        cout << "Test sphere wrong!\n";
    }
    return 0;
}