#ifndef CONTACT_H_
#define CONTACT_H_

#include "ARRAY.h"
#include "vmath.h"
#include <vector>

class Rigid_Geometry;

class Contact {
public:
    static enum {FRICTION_DIM = 6};

    Contact()
    : a(0), b(0), D(FRICTION_DIM), friction(FRICTION_DIM) {
    }
    static void contact_handling(std::vector<Contact>& contacts);
    double mu;
    Rigid_Geometry *a, *b;
    Vector3d p, n, ra, rb, u;
    double support;
    SimLib::ARRAY<1,Vector3d> D;
    SimLib::ARRAY<1,double> friction;
};



#endif