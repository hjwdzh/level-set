#ifndef IMPLICIT_CUBE_H_
#define IMPLICIT_CUBE_H_

#include "VECTOR.h"
#include "RANGE.h"
#include "IMPLICIT_OBJECT.h"

namespace SimLib {
    
template<class T>
class IMPLICIT_CUBE: public IMPLICIT_OBJECT<T>{
public:
    typedef VECTOR<T,3> TV;
    IMPLICIT_CUBE(){}
    IMPLICIT_CUBE(const RANGE<TV>& a)
    : box(a)
    {}
    std::pair<T, TV> Intersect(const TV& p) {
        double d = 1e30;
        int t = 0;
        TV normal = TV();
        if (!box.Contain(p))
            return make_pair(d,normal);
        d = -d;
        if (box.min(1) - p(1) > d) {
            d = box.min(1) - p(1);
            t = 1;
        }
        if (p(1) - box.max(1) > d) {
            d = p(1) - box.max(1);
            t = 2;
        }
        if (box.min(2) - p(2) > d) {
            d = box.min(2) - p(2);
            t = 3;
        }
        if (p(2) - box.max(2) > d) {
            d = p(2) - box.max(2);
            t = 4;
        }
        if (box.min(3) - p(3) > d) {
            d = box.min(3) - p(3);
            t = 5;
        }
        if (p(3) - box.max(3) > d) {
            d = p(3) - box.max(3);
            t = 6;
        }
        switch (t) {
            case 1:
                normal = TV(-1,0,0);
                break;
            case 2:
                normal = TV(1,0,0);
                break;
            case 3:
                normal = TV(0,-1,0);
                break;
            case 4:
                normal = TV(0,1,0);
                break;
            case 5:
                normal = TV(0,0,-1);
                break;
            case 6:
                normal = TV(0,0,1);
                break;
                
            default:
                break;
        }
        return make_pair(d,normal);
    }
    RANGE<TV> box;
};
}

#endif