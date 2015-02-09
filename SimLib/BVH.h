#ifndef BVH_H_
#define BVH_H_

#include <vector>
#include "KDOP.h"

class Contact;

namespace SimLib {

template<int d, class T>
class BVH {
public:
    BVH()
    : left(0), right(0), bv(0)
    {}
    ~BVH() {
        clear();
    }
    void clear() {
        if (left)
            delete left;
        if (right)
            delete right;
        delete bv;
        left = 0;
        right = 0;
        bv = 0;
    }
    void updateBVH(std::vector<BV<T>*>& bvs, int dim, int l, int r);
    void collide_detection(std::vector<Contact>* contacts=0);
    void collide_detect(BVH*, BVH*, std::vector<Contact>* contacts=0);
    int axis;
    BVH *left, *right;
    BV<T>* bv;
    int num;
};
    
}

#endif