#ifndef ARRAY_H_
#define ARRAY_H_

#include <vector>
#include <assert.h>
#include "VECTOR.h"
#include "RANGE.h"

namespace SimLib {

template<int d, class TV>
class ARRAY {
    typedef VECTOR<int, d> TV_INT;
public:
    ARRAY()
    : bx(1), by(1), bz(1)
    {}
    ARRAY(int x)
    : bx(1){
        assert(d == 1);
        dim = VECTOR<int, d>(x);
        data.resize(x);
    }
    ARRAY(int x, int y)
    : bx(1), by(1) {
        assert(d == 2);
        dim = VECTOR<int, d>(x, y);
        data.resize(x * y);
    }
    ARRAY(int x, int y, int z)
    : bx(1), by(1), bz(1) {
        assert(d == 3);
        dim = VECTOR<int, d>(x,y,z);
        data.resize(x * y * z);
    }
    ARRAY(const TV_INT& r)
    : bx(1), by(1), bz(1) {
        dim = r;
        data.resize(r.vol());
    }
    ARRAY(const RANGE<TV_INT>& r) {
        bx = r.min(1);
        by = r.min(2);
        bz = r.min(3);
        dim = r.max - r.min + TV_INT(1, 1, 1);
        data.resize(dim.vol());
    }
    void Resize(const RANGE<TV_INT>& r) {
        bx = r.min(1);
        by = r.min(2);
        bz = r.min(3);
        dim = r.max - r.min + TV_INT(1, 1, 1);
        data.resize(dim.vol());        
    }
    TV& operator()(int x) {
        assert(d == 1);
        return data[x - bx];
    }
    const TV& operator() (int x) const {
        assert(d == 1);
        return data[x - bx];
    }
    TV& operator()(int x, int y) {
        assert(d == 2);
        return data[(x - bx) * dim(2) + y - by];
    }
    const TV& operator() (int x, int y) const {
        assert(d == 2);
        return data[(x - bx) * dim(2) + y - by];
    }
    TV& operator()(int x, int y, int z) {
        assert(d == 3);
        return data[(x - bx) * dim(2) * dim(3) + (y - by) * dim(3) + (z - bz)];
    }
    ARRAY& operator-=(const TV& f) {
        for (int i = 0; i < data.size(); ++i)
            data[i] -= f;
        return (*this);
    }
    const TV& operator() (int x, int y, int z) const {
        assert(d == 3);
        return data[(x - bx) * dim(2) * dim(3) + (y - by) * dim(3) + (z - bz)];
    }
    TV& operator()(const TV_INT& index) {
        if (d == 1)
            return operator()(index(1));
        if (d == 2)
            return operator()(index(1), index(2));
        if (d == 3)
            return operator()(index(1), index(2), index(3));
    }
    void Fill(TV r) {
        for (int i = 0; i < data.size(); ++i)
            data[i] = r;
    }
    RANGE<TV_INT> domain() const {
        return RANGE<TV_INT>(TV_INT(bx,by,bz), TV_INT(bx,by,bz) + dim - TV_INT(1,1,1));
    }
    VECTOR<int, d> dim;
    std::vector<TV> data;
    int bx, by, bz;
};
    
}

#endif