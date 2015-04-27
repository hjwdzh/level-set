#ifndef CONTACT_H_
#define CONTACT_H_

#include "ARRAY.h"
#include "vmath.h"
#include <vector>

class Rigid_Geometry;

class Contact {
public:
    Contact()
    : a(0), b(0){
    }
    static void contact_handling(std::vector<Contact>& contacts);
    bool collide_handling(double k = -100, bool solve = true);
    double mu, kr;
    Rigid_Geometry *a, *b;
    Vector3d p, n, ra, rb, u, t1, t2;
    double support;
};



#endif