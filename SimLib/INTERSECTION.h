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
            TV h = TV::Cross_Product(ray.d,e2);
            T a = e1.Dot_Product(h);
            T f = 1 / a;
            TV s = ray.o - triangle.a;
            T u = f * s.Dot_Product(h);
            if (u < -1e-4 || u > 1.0001)
                return false;
            TV Q = TV::Cross_Product(s, e1);
            T v = f * ray.d.Dot_Product(Q);
            if (v < -1e-4 || u + v > 1.0001)
                return false;
            T dt = f * e2.Dot_Product(Q);
            if (thickness_over_two > 0) {
                TV normal = TV::Cross_Product(e1,e2).Normal();
                T thickness = thickness_over_two * abs(ray.d.Dot_Product(normal));
                if (dt > thickness)
                    dt -= thickness;
            }
            if (dt > -1e-2 && (ray.semi_infinite || dt < ray.t_max + 1e-2)) {
                ray.intersect = dt;
                return true;
            }
            
            return false;
        }
    };
}

#endif
