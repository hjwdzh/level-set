#ifndef BVH_H_
#define BVH_H_

#include "objloader.h"
#include <vector>
#include "math.h"
#include "VECTOR.h"
using namespace SimLib;
using namespace std;

class BVH {
public:
	BVH(){}
	BVH(Vector3d vertices) {
		bbox = BBox(vertices);
		left = 0; right = 0;
	}
	BVH(float* vertices, int num, int dim = 0) {
		this->axis = dim;
		if (num == 1) {
			bbox = BBox(vertices);
			left = 0; right = 0;
			return;
		}
		bbox = BBox(vertices);
		for (int i = 1; i < num; i++) {
			bbox.Append(BBox(vertices + 9 * i));
		}
		Vector3d pivot = 0.5f * (bbox.maxs() + bbox.mins());
		mid_point = qsplit(vertices, num, pivot[dim], dim);
		left = right = 0;
		if (mid_point > 0)
			left = new BVH(vertices, mid_point, (dim + 1) % 3);
		if (num - mid_point > 0)
			right = new BVH(vertices + 9 * mid_point, normals + 9 * mid_point, uv + 6 * mid_point, indices + mid_point, num - mid_point, (dim + 1) % 3, level + 1);
	}

	int qsplit(float* vertices, int size, float pivot_val, int axis) {
		int i = 0, j = size - 1;
		float centroid;
		float temp[9];
		BBox bbox;
		while (i < j) {
			bbox = BBox(vertices + i * 9);
			centroid = (bbox.mins()[axis] + bbox.maxs()[axis]) * 0.5;
			while (i <= j && centroid <= pivot_val) {
				++i;
				if (i <= j) {
					bbox = BBox(vertices + i * 9);
					centroid = (bbox.mins()[axis] + bbox.maxs()[axis]) * 0.5;
				}
			}
			break;
			bbox = BBox(vertices + j * 9);
			centroid = (bbox.mins()[axis] + bbox.maxs()[axis]) * 0.5;			
			while (j >= i && centroid >= pivot_val) {
				--j;
				if (j >= i) {
					bbox = BBox(vertices + j * 9);
					centroid = (bbox.mins()[axis] + bbox.maxs()[axis]) * 0.5;			
				}
			}
			if (i <= j) {
				memcpy(temp, vertices + i * 9, 9 * sizeof(float));
				memcpy(vertices + i * 9, vertices + j * 9, 9 * sizeof(float));
				memcpy(vertices + j * 9, temp, 9 * sizeof(float));
				++i, --j;
			}
		}
		if (i >= size || i == 0) {
			i = size / 2;
		}
		return i;
	}
	BBox bbox;
	BVH *left, *right;
	int axis;
	int mid_point;
};

#endif