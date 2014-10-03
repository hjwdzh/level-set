#ifndef VECTOR_H_
#define VECTOR_H_

#include <assert.h>
#include <math.h>
#include <algorithm>

namespace SimLib {
template<class T, int d>
class VECTOR {
public:
	typedef T SCALAR;
	enum WORKAROUND1 {dimension=d};
	VECTOR() {
		data = new T[d];
		for (int i = 0; i < d; ++i)
			data[i] = (T)0;
	}
	VECTOR(const VECTOR& a) {
		data = new T[d];
		for (int i = 0; i < d; ++i)
			data[i] = a(i+1);
	}
	VECTOR(const T& a) {
		assert(d == 1);
		data = new T[d];
		data[0] = a;
	}
	VECTOR(const T& a, const T& b) {
		assert(d == 2);
		data = new T[d];
		data[0] = a;
		data[1] = b;
	}
	T vol() const {
        T v = 1;
        for (int i = 0; i < d; ++i)
            v *= data[i];
        return v;
    }
    T Magnitude() const {
        T v = 0;
        for (int i = 0; i < d; ++i)
            v += data[i] * data[i];
        return sqrt(v);
    }
    T Magnitude_Squared() const {
        T v = 0;
        for (int i = 0; i < d; ++i)
            v += data[i] * data[i];
        return v;
    }
	VECTOR(const T& a, const T& b, const T& c) {
		assert(d == 3);
		data = new T[d];
		data[0] = a;
		data[1] = b;
		data[2] = c;
	}
	~VECTOR() {
		delete[] data;
	}
	VECTOR& operator=(const VECTOR& v) {
		for (int i = 0; i < d; ++i)
			data[i] = v(i+1);
		return (*this);
	}
	T& operator()(int x) {
		assert(x <= d);
		return data[x + 1];
	}
	const T& operator()(int x) const {
		assert(x <= d);
		return data[x + 1];
	}
	VECTOR& operator+=(const VECTOR& a) {
		for (int i = 0; i < d; ++i)
			data[i] += a(i+1);
		return (*this);
	}
	VECTOR& operator*=(const VECTOR& a) {
		for (int i = 0; i < d; ++i)
			data[i] *= a(i+1);
		return (*this);
	}
	VECTOR& operator-=(const VECTOR& a) {
		for (int i = 0; i < d; ++i)
			data[i] -= a(i+1);
		return (*this);
	}
	VECTOR operator-() {
		VECTOR v;
		for (int i = 0; i < d; ++i)
			v(i+1) = -data[i];
		return v;
	}
	VECTOR& operator/=(const VECTOR& a) {
		for (int i = 0; i < d; ++i)
			data[i] /= a(i+1);
		return (*this);
	}
	T Min() const {
		T min_num = 1e30;
		for (int i = 0; i < d; ++i)
			if (data[i] < min_num)
				min_num = data[i];
		return min_num;
	}
    T Max() const {
        T max_num = -1e30;
        for (int i = 0; i < d; ++i)
            if (data[i] > max_num)
                max_num = data[i];
        return max_num;
    }
    T Dot_Product(const VECTOR& a) const {
        return ((*this) * a).Sum();
    }    
    VECTOR Get_Max(const VECTOR& a) const {
        VECTOR v = (*this);
        for (int i = 1; i <= d; ++i)
            v(i) = std::max(v(i), a(i));
        return v;
    }
    VECTOR Get_Min(const VECTOR& a) const {
        VECTOR v = (*this);
        for (int i = 1; i <= d; ++i)
            v(i) = std::min(v(i), a(i));
        return v;
    }
    T Sum() const {
        T v = 0;
        for (int i = 0; i < d; ++i)
            v += data[i];
        return v;
    }
    static VECTOR Cross_Product(const VECTOR& v1, const VECTOR& v2) {
        assert(d == 3);
        return VECTOR(v1(2)*v2(3)-v1(3)*v2(2),v1(3)*v2(1)-v1(1)*v2(3),v1(1)*v2(2)-v1(2)*v2(1));
    }
	T* data;
};
template <class T,int d>
    inline VECTOR<T,d> operator+(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a(i) + b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator-(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a(i) - b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator*(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a(i) * b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator/(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a(i) / b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator+(const T a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a + b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator-(const T a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a - b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator*(const VECTOR<T, d>& b, const T a) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a * b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator*(const T a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a * b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator/(const T a, const VECTOR<T, d>& b) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = a / b(i);
        return res;
    }
template <class T,int d>
    inline VECTOR<T,d> operator/(const VECTOR<T, d>& b, const T a) {
        VECTOR<T, d> res;
        for (int i = 1; i <= d; ++i)
            res(i) = b(i) / a;
        return res;
    }
template <class T,int d>
    inline bool operator<(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        for (int i = 1; i <= d; ++i)
            if (a(i) >= b(i))
                return false;
        return true;
    }
template <class T,int d>
    inline bool operator==(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        for (int i = 1; i <= d; ++i)
            if (a(i) != b(i))
                return false;
        return true;
    }
template <class T,int d>
    inline bool operator<=(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        for (int i = 1; i <= d; ++i)
            if (a(i) > b(i))
                return false;
        return true;
    }
template <class T,int d>
    inline bool operator>(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        return !(a <= b);
    }
template <class T,int d>
    inline bool operator>=(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
        return !(a < b);
    }
}
#endif