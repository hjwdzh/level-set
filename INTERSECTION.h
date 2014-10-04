#ifndef INTERSECTION_H_
#define INTERSECTION_H_

#include "VECTOR.h"
#include "SEGMENT.h"
#include "RAY.h"
#include <math.h>

namespace SimLib {
template<class T>
class INTERSECTION{
public:
    typedef VECTOR<T,3> TV;
    static bool Intersects(const SEGMENT<T>& segment,const TRIANGLE<T>& triangle,const T thickness_over_two = 0)
    {
        RAY<T> ray(segment.a,segment.b-segment.a);
        ray.semi_infinite=false;ray.t_max=(segment.b-segment.a).Magnitude();
        return INTERSECTION::Intersects(ray,triangle,thickness_over_two);
    }
    static bool Intersects(RAY<T>& ray,const TRIANGLE<T>& triangle, const T thickness_over_two = 0)
    {
        TV e1 = triangle.b - triangle.a;
        TV e2 = triangle.c - triangle.a;
        TV p = TV::Cross_Product(ray.d,e2);
        float det = e1.Dot_Product(p);
        TV t;
        if (det > 0)
            t = ray.o - triangle.a;
        else {
            t = triangle.a - ray.o;
            det = -det;
        }
        if (det < 1e-6)
            return false;
        T u = t.Dot_Product(p);
        if (u < 0 || u > det)
            return false;
        TV Q = TV::Cross_Product(t, e1);
        T v = ray.d.Dot_Product(Q);
        if (v < 0 || u + v > det)
            return false;
        T dt = e2.Dot_Product(Q);
        if (thickness_over_two > 0) {
            TV normal = TV::Cross_Product(e1,e2).Normal();
            dt += abs(ray.d.Dot_Product(normal));
        }
        if (ray.semi_infinite || dt < ray.t_max) {
            ray.intersect = dt;
            return true;
        }
        
        return false;
    }
};
}

#endif
