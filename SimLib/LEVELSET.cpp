#include "LEVELSET.h"
#include "BOUNDARY.h"
#include "math.h"
#include <iostream>
#include <fstream>
using namespace SimLib;

template<class T_GRID>
void LEVELSET<T_GRID>::Fast_Marching_Method(TRIANGULATED_SURFACE<float>& tris, ARRAY<3,int>& closest_index, const T stopping_distance) {
    heap_index.clear();
    array_ind = ARRAY<3, int>(phi.dim);
    for (int i = 1; i <= phi.dim(1); ++i) {
        for (int j = 1; j <= phi.dim(2); ++j) {
            for (int k = 1; k <= phi.dim(3); ++k) {
                if (phi(i,j,k) > 1e20)
                    continue;
                heap_index.push_back(TV_INT(i,j,k));
                if (phi(i,j,k) > -1e20) {
                    array_ind(i,j,k) = -((int)heap_index.size());
                } else {
                    array_ind(i,j,k) = (int)heap_index.size();
                }
            }
        }
    }
    closestIndex = &closest_index;
    triList = &tris;
    for (int i = (int)heap_index.size() / 2 - 1; i >= 0; --i) {
        Down_Adjust(i);
    }
    while (!heap_index.empty()) {
        TV_INT ind = Extract();
        Extend_Distance(ind);
    }
}

template<class T_GRID>
void LEVELSET<T_GRID>::Down_Adjust(int i) {
    int j = (i << 1) + 1;
    TV_INT ind = heap_index[i];
    int k = array_ind(heap_index[i]);
    T val = phi(ind);
    while (j < heap_index.size()) {
        if (j + 1 < heap_index.size() && phi(heap_index[j + 1]) > phi(heap_index[j])) {
            ++j;
        }
        if (phi(heap_index[j]) > val) {
            int k1 = array_ind(heap_index[j]);
            array_ind(heap_index[j]) = k;
            k = k1;
            heap_index[i] = heap_index[j];
            i = j;
            j = (i << 1) + 1;
        } else {
            break;
        }
    }
    heap_index[i] = ind;
    array_ind(ind) = k;
}

template<class T_GRID>
void LEVELSET<T_GRID>::Up_Adjust(int i) {
    int j = (i - 1) >> 1;
    TV_INT ind = heap_index[i];
    int k = array_ind(heap_index[i]);
    T val = phi(ind);
    while (j >= 0) {
        if (val > phi(heap_index[j])) {
            int k1 = array_ind(heap_index[j]);
            array_ind(heap_index[j]) = k;
            k = k1;
            heap_index[i] = heap_index[j];
            i = j;
            j = (i - 1) >> 1;
        } else {
            break;
        }
    }
    heap_index[i] = ind;
    array_ind(ind) = k;
}

template<class T_GRID>
typename LEVELSET<T_GRID>::TV_INT LEVELSET<T_GRID>::Extract() {
    TV_INT ind = heap_index[0];
    heap_index[0] = heap_index.back();
    int k = array_ind(ind);
    array_ind(heap_index.back()) = k;
    array_ind(ind) = 0;
    heap_index.pop_back();
    if (!heap_index.empty())
        Down_Adjust(0);
    return ind;
}

template<class T_GRID>
void LEVELSET<T_GRID>::Extend_Distance(TV_INT& ind) {
    for (int i = -1; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
            for (int k = -1; k < 2; ++k) {
                if (i == 0 && j == 0 && k == 0)
                    continue;
                TV_INT temp_ind = ind + TV_INT(i,j,k);
                int t = array_ind(temp_ind);
                if (t != 0) {
                    Estimate_Distance(temp_ind, ind);
                    if (t < 0)
                        t = -t;
                    if (t > 0)
                        Up_Adjust(t - 1);
                }
            }
        }
    }
}

template<class T_GRID>
void LEVELSET<T_GRID>::Estimate_Distance(TV_INT& ind, TV_INT& parent_ind) {
    TV x = grid.X(ind);
    int tr = (*closestIndex)(parent_ind);
    TV closest_point = triList->triangle_list[tr-1].Closest_Point(x);
    T dis = (x-closest_point).Magnitude();
    T t = -phi(ind);
    if (t > dis) {
        phi(ind) = -dis;
        int k = array_ind(ind);
        (*closestIndex)(ind) = tr;
        if (k < 0)
            array_ind(ind) = -k;
    }
}
template<class T_GRID>
std::pair<typename LEVELSET<T_GRID>::T, typename LEVELSET<T_GRID>::TV> LEVELSET<T_GRID>::Intersect(const TV& p) {
    if (!grid.Include(p) || !grid.Include(p + grid.dx)) {
        return std::make_pair(1e30, TV(0,0,0));
    }
    T depth = grid.Interpolate(phi, p);
    TV normal = grid.Gradient(phi, p);
    return std::make_pair(depth, normal);
}

template<class T_GRID>
std::pair<typename LEVELSET<T_GRID>::T, typename LEVELSET<T_GRID>::TV> LEVELSET<T_GRID>::Intersect(const TV& p1, const TV& p2, float& portion) {
    std::pair<T, TV> m_intersect;
    m_intersect.first = 1e30;
    TV d = (p2 - p1) * grid.one_over_dX;
    float t = d.Max();
    for (int i = 0; i < t; ++i) {
        std::pair<T, TV> intersect = Intersect(p1 + (p2 - p1) * (i / t));
        if (intersect.first < m_intersect.first) {
            m_intersect = intersect;
            portion = i / t;
        }
    }
    return m_intersect;
}


template class SimLib::LEVELSET<GRID<VECTOR<float,3> > >;