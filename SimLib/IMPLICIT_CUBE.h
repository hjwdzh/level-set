#ifndef IMPLICIT_CUBE_H_
#define IMPLICIT_CUBE_H_

#include "VECTOR.h"
#include "RANGE.h"
#include "IMPLICIT_OBJECT.h"
#include <math.h>
#include <algorithm>

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
        if (box.getMin(1) - p(1) > d) {
            d = box.getMin(1) - p(1);
            t = 1;
        }
        if (p(1) - box.getMax(1) > d) {
            d = p(1) - box.getMax(1);
            t = 2;
        }
        if (box.getMin(2) - p(2) > d) {
            d = box.getMin(2) - p(2);
            t = 3;
        }
        if (p(2) - box.getMax(2) > d) {
            d = p(2) - box.getMax(2);
            t = 4;
        }
        if (box.getMin(3) - p(3) > d) {
            d = box.getMin(3) - p(3);
            t = 5;
        }
        if (p(3) - box.getMax(3) > d) {
            d = p(3) - box.getMax(3);
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
    std::pair<T, TV> Intersect(const TV& p1, const TV& p2, float& portion) {
        if (box.Contain(p1)) {
            portion = 0;
            return Intersect(p1);
        } else
            if (box.Contain(p2)) {
                portion = 1;
                return Intersect(p2);
            }
        TV d = p2 - p1;
        double minx, maxx, miny, maxy, minz, maxz, t1, t2;
        if (p1(1) == p2(1)) {
            if (p1(1) < box.getMin(1) || p1(1) > box.getMax(1)) {
                return make_pair(1e30, TV());
            } else {
                minx = 0;
                maxx = 1;
            }
        }
        t1 = (box.getMin(1) - p1(1)) / (p2(1) - p1(1));
        t2 = (box.getMax(1) - p1(1)) / (p2(1) - p1(1));
        minx = (t1 < t2) ? t1 : t2;
        maxx = (t1 < t2) ? t2 : t1;
        if (p1(2) == p2(2)) {
            if (p1(2) < box.getMin(2) || p1(2) > box.getMax(2)) {
                return make_pair(1e30, TV());
            } else {
                miny = 0;
                maxy = 1;
            }
        }
        t1 = (box.getMin(2) - p1(2)) / (p2(2) - p1(2));
        t2 = (box.getMax(2) - p1(2)) / (p2(2) - p1(2));
        miny = (t1 < t2) ? t1 : t2;
        maxy = (t1 < t2) ? t2 : t1;
        if (p1(3) == p2(3)) {
            if (p1(3) < box.getMin(3) || p1(3) > box.getMax(3)) {
                return make_pair(1e30, TV());
            } else {
                minz = 0;
                maxz = 1;
            }
        }
        t1 = (box.getMin(3) - p1(3)) / (p2(3) - p1(3));
        t2 = (box.getMax(3) - p1(3)) / (p2(3) - p1(3));
        minz = (t1 < t2) ? t1 : t2;
        maxz = (t1 < t2) ? t2 : t1;
        
        double m1 = max(0.0, max(max(minx, miny), minz));
        double m2 = min(1.0, min(min(maxx, maxy), maxz));
        if (m1 > m2) {
            return make_pair(1e30, TV());
        }
        portion = (m2 + m1) * 0.5;
        TV p = (p2 - p1) * portion + p1;
        TV minp = p - box.min;
        TV maxp = box.max - p;
        TV mdis = minp.Get_Min(maxp);
        t1 = mdis(1) * mdis(1) + mdis(2) * mdis(2);
        t2 = mdis(1) * mdis(1) + mdis(3) * mdis(3);
        double t3 = mdis(2) * mdis(2) + mdis(3) * mdis(3);
        std::pair<T, TV> intersection;
        TV n;
        TV axis = (p2 - p1 + TV(1.18e-6, 2.39e-6, 3.21e-6)).Normal();
        if (t1 < t2 && t1 < t3) {
            intersection.first = -sqrt(t1);
            n = TV::Cross_Product(TV(0, 0, 1), axis);
            if (min(p(1) - box.min(1) + n(1) * 1e-6, box.max(1) - p(1) - 1e-6 * n(1)) > mdis(1))
                n = -n;
        } else if (t2 < t1 && t2 < t3) {
            intersection.first = -sqrt(t1);
            n = TV::Cross_Product(TV(0, 1, 0), axis);
            if (min(p(1) - box.min(1) + n(1) * 1e-6, box.max(1) - p(1) - 1e-6 * n(1)) > mdis(1))
                n = -n;
        } else {
            intersection.first = -sqrt(t3);
            n = TV::Cross_Product(TV(1, 0, 0), axis);
            if (min(p(2) - box.min(2) + n(2) * 1e-6, box.max(2) - p(2) - 1e-6 * n(2)) > mdis(2))
                n = -n;
        }
        intersection.second = n;
        return intersection;
    }
    RANGE<TV> box;
};
}

#endif