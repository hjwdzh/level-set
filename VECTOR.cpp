#include "VECTOR.h"

using namespace SimLib;

template<class T, int d>
const VECTOR<T, d>& operator+(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a(i) + b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator-(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a(i) - b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator*(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a(i) * b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator/(const VECTOR<T, d>& a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a(i) / b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator+(const T a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a + b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator-(const T a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a - b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator*(const T a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a * b(i);
	return res;
}

template<class T, int d>
const VECTOR<T, d>& operator/(const T a, const VECTOR<T, d>& b) {
	VECTOR<T, d> res;
	for (int i = 1; i <= d; ++i)
		res(i) = a / b(i);
	return res;
}

template class VECTOR<float, 3>;