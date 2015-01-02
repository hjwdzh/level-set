#ifndef TRI_HPP
#define TRI_HPP

#include <map>
#include <vector>
#include <string>
#include "VECTOR.h"
#include "LEVELSET.h"
#include "TRIANGULATED_SURFACE.h"
#include "LEVELSET_MAKER.h"
#include "vmath.h"

using namespace std;
using namespace SimLib;

struct TriData {
    GRID<VECTOR<float, 3> >* grid;
    ARRAY<3, float>* phi;
    IMPLICIT_OBJECT<float>* implicit_object;
    TRIANGULATED_SURFACE<float>* triangles;
    Vector3d x;
};

class TriManager {
public:
    typedef VECTOR<float,3> TV;
    typedef VECTOR<int,3> TV_INT;
	static map<string,int> nameManager;
	static vector<TriData> triResource;
	static TriData createTriMeshAndLevelSet(const char* name, bool createLevelSet = true) {
		string n = string(name);
		map<string,int>::iterator it = nameManager.find(n);
		int texid;
		if (it == nameManager.end()) {
            TriData data;
            data.triangles = new TRIANGULATED_SURFACE<float>();
            data.triangles->loadOBJ(name);
            data.x = data.triangles->Update_Bounding_Box_And_Gravity_Center();
            RANGE<TV> range = data.triangles->bounding_box;
            range.min -= TV(0.2, 0.2, 0.2);
            range.max += TV(0.2, 0.2, 0.2);
            data.grid = new GRID<TV>(TV_INT(40,40,40), range);
            data.phi = new ARRAY<3, float>(data.grid->Domain_Indices());
            if (createLevelSet) {
                data.implicit_object = new LEVELSET<GRID<VECTOR<float, 3> > >(*data.grid, *data.phi);
                SimLib::LEVELSET_MAKER<float> level_maker;
                ARRAY<3, int> closest_index;
                level_maker.Compute_Level_Set(*data.triangles, *data.grid, *data.phi, closest_index);
                ((LEVELSET<GRID<VECTOR<float, 3> > >*)data.implicit_object)->Fast_Marching_Method(*data.triangles, closest_index);
            } else {
                data.implicit_object = 0;
            }
            triResource.push_back(data);
			texid = (int)triResource.size() - 1;
			nameManager.insert(pair<string,int>(n, texid));
		} else {
			texid = it->second;
		}
		return triResource[texid];
	}
};

#endif