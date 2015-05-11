#include "BVH.h"
#include "Rigid_Geometry.h"

using namespace SimLib;

template<int d, class T>
void BVH<d,T>::updateBVH(std::vector<BV<T>*>& bvs, int dim, int l, int r) {
    left = 0, right = 0;
    axis = dim;
    if (l < 0) {
        l = 0;
        r = (int)bvs.size() - 1;
    }
    num = r - l + 1;
    bv = new KDOP<3, float>();
    bv->include(&bvs[l], &bvs[r]);
    if (l == r) {
        bv->rgd = bvs[l]->rgd;
        return;
    }
    T pivot = (*bv)(dim);
    int i = l, j = r;
    while (i < j) {
        while ((*bvs[i])(dim) < pivot) {
            ++i;
        }
        while ((*bvs[j])(dim) > pivot) {
            --j;
        }
        if (i <= j) {
            BV<T>* bv = bvs[i];
            bvs[i] = bvs[j];
            bvs[j] = bv;
            ++i, --j;
        }
    }
    if (i > r)
        i = r;
    left = new BVH();
    left->updateBVH(bvs, dim % d + 1, l, i - 1);
    right = new BVH();
    right->updateBVH(bvs, dim % d + 1, i, r);
}

template<int d, class T>
void BVH<d,T>::collide_detection(std::vector<Contact>* contacts) {
    if (num < 2)
        return;
    if (left && right)
        collide_detect(left, right, contacts);
    if (left)
        left->collide_detection(contacts);
    if (right)
        right->collide_detection(contacts);
}

template<int d, class T>
void BVH<d,T>::collide_detect(BVH<d, T> *a, BVH<d, T> *b, std::vector<Contact>* contacts) {
    if (!a->bv->intersect(b->bv))
        return;
    if (a->num == 1 && b->num == 1 && contacts) {
        a->bv->rgd->collide_detection(b->bv->rgd, contacts);
//        b->bv->rgd->collide_detection(a->bv->rgd, contacts);
        return;
    }
    if (a->num > b->num) {
        if (a->left) {
            collide_detect(a->left, b, contacts);
        }
        if (a->right) {
            collide_detect(a->right, b, contacts);
        }
    } else {
        if (b->left) {
            collide_detect(a, b->left, contacts);
        }
        if (b->right) {
            collide_detect(a, b->right, contacts);
        }
    }
}

template class SimLib::BVH<3, float>;