#include "LEVELSET.h"
#include "BOUNDARY.h"
#include "math.h"
#include <iostream>
using namespace SimLib;

template<class T_GRID>
void LEVELSET<T_GRID>::Fast_Marching_Method(const T stopping_distance) {
    heap_index.clear();
    array_ind = ARRAY<3, int>(phi.dim);
    for (int i = 1; i <= phi.dim(1); ++i) {
        for (int j = 1; j <= phi.dim(2); ++j) {
            for (int k = 1; k <= phi.dim(3); ++k) {
                if (phi(i,j,k) < -1e20) {
                    T d1 = fmax(phi(i-1,j,k),phi(i+1,j,k))-grid.dx(1);
                    T d2 = fmax(phi(i,j-1,k),phi(i,j+1,k))-grid.dx(2);
                    T d3 = fmax(phi(i,j,k-1),phi(i,j,k+1))-grid.dx(3);
                    phi(i,j,k) = fmax(fmax(d1, d2), d3);
                    heap_index.push_back(TV_INT(i,j,k));
                    array_ind(i,j,k) = -((int)heap_index.size());
                }
            }
        }
    }
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
    while (j < heap_index.size()) {
        if (j + 1 < heap_index.size() && phi(heap_index[j + 1]) > phi(heap_index[j])) {
            ++j;
        }
        if (phi(heap_index[j]) > phi(heap_index[i])) {
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
    while (j >= 0) {
        if (phi(heap_index[i]) > phi(heap_index[j])) {
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
    if (k < 0) {
        Estimate_Distance(ind);
    }
    array_ind(heap_index.back()) = k;
    array_ind(ind) = 0;
    heap_index.pop_back();
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
                    Estimate_Distance(temp_ind);
                    if (t < 0)
                        t = -t;
                    Up_Adjust(t - 1);
                }
            }
        }
    }
}

template<class T_GRID>
void LEVELSET<T_GRID>::Estimate_Distance(TV_INT& ind) {
    T phi1 = fmin(-phi(ind + TV_INT(-1, 0, 0)), -phi(ind + TV_INT(1, 0, 0)));
    T phi2 = fmin(-phi(ind + TV_INT(0, -1, 0)), -phi(ind + TV_INT(0, 1, 0)));
    T phi3 = fmin(-phi(ind + TV_INT(0, 0, -1)), -phi(ind + TV_INT(0, 0, 1)));
    T dx1 = 1 / (grid.dx(1) * grid.dx(1));
    T dx2 = 1 / (grid.dx(2) * grid.dx(2));
    T dx3 = 1 / (grid.dx(3) * grid.dx(3));
    T A = dx1 + dx2 + dx3;
    T B = -2 * (phi1 * dx1 + phi2 * dx2 + phi3 * dx3);
    T C = phi1 * phi1 * dx1 + phi2 * phi2 * dx2 + phi3 * phi3 * dx3;
    T r = (-B + sqrt(B*B-4*A*C))/(2*A);
    T t = -phi(ind);
    if (t > r) {
        phi(ind) = -r;
        int k = array_ind(ind);
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


template class SimLib::LEVELSET<GRID<VECTOR<float,3> > >;