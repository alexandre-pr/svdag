#include "vec3.h"

#pragma once

#include <cmath>
#include <iostream>

/// Vector in 3 dimensions, with basics operators overloaded.
template <typename T>
class Mat3 {
public:
    inline Mat3() {
        m_p = Vec3<Vec3<T>>(Vec3<T>(),
            Vec3<T>(),
            Vec3<T>());
    }

    inline Mat3(Vec3<T> p0, Vec3<T> p1, Vec3<T> p2) {
        m_p = Vec3<Vec3<T>>(p0,
            p1,
            p2);
    };

    inline Mat3(const Mat3& v) {
        init(v[0], v[1], v[2]);
    }

    ~Mat3() {}

    inline Vec3<T>& operator[] (int Index) {
        return (m_p[Index]);
    };

    inline const Vec3<T>& operator[] (int Index) const {
        return (m_p[Index]);
    };

    inline Mat3& operator= (const Mat3& p) {
        m_p[0] = p[0];
        m_p[1] = p[1];
        m_p[2] = p[2];
        return (*this);
    };

    inline Mat3& operator+= (const Mat3& p) {
        m_p[0] += p[0];
        m_p[1] += p[1];
        m_p[2] += p[2];
        return (*this);
    };

    inline Mat3& operator-= (const Mat3& p) {
        m_p[0] -= p[0];
        m_p[1] -= p[1];
        m_p[2] -= p[2];
        return (*this);
    };

    inline Mat3& operator*= (T s) {
        m_p[0] *= s;
        m_p[1] *= s;
        m_p[2] *= s;
        return (*this);
    };

    inline Mat3& operator/= (T s) {
        m_p[0] /= s;
        m_p[1] /= s;
        m_p[2] /= s;
        return (*this);
    };

    inline Mat3 operator+ (const Mat3& p) const {
        Vec3<T> res;
        res[0] = m_p[0] + p[0];
        res[1] = m_p[1] + p[1];
        res[2] = m_p[2] + p[2];
        return (res);
    };

    inline Mat3 operator- (const Mat3& p) const {
        Vec3<T> res;
        res[0] = m_p[0] - p[0];
        res[1] = m_p[1] - p[1];
        res[2] = m_p[2] - p[2];
        return (res);
    };

    inline Mat3 operator- () const {
        Vec3<T> res;
        res[0] = -m_p[0];
        res[1] = -m_p[1];
        res[2] = -m_p[2];
        return (res);
    };

    inline Vec3<T> operator* (const Vec3<T>& p) const {
        Vec3<T> res;
        res[0] = dot(m_p[0],p);
        res[1] = dot(m_p[1],p);
        res[2] = dot(m_p[2],p);
        return (res);
    };

    inline Mat3 operator* (const Mat3& p) const {
        Mat3 res;
        Mat3 m = p.transpose();
        res[0] = m_p * m[0];
        res[1] = m_p * m[1];
        res[2] = m_p * m[2];
        return (res);
    };

    inline Mat3 operator* (T s) const {
        Vec3<T> res;
        res[0] = m_p[0] * s;
        res[1] = m_p[1] * s;
        res[2] = m_p[2] * s;
        return (res);
    };

    inline Mat3 operator/ (const Mat3& p) const {
        Vec3<T> res;
        res[0] = m_p[0] / p[0];
        res[1] = m_p[1] / p[1];
        res[2] = m_p[2] / p[2];
        return (res);
    };

    inline Mat3 operator/ (T s) const {
        Vec3<T> res;
        res[0] = m_p[0] / s;
        res[1] = m_p[1] / s;
        res[2] = m_p[2] / s;
        return (res);
    };

    inline bool operator == (const Mat3& a) const {
        return(m_p[0] == a[0] && m_p[1] == a[1] && m_p[2] == a[2]);
    };

    inline bool operator != (const Mat3& a) const {
        return(m_p[0] != a[0] || m_p[1] != a[1] || m_p[2] != a[2]);
    };

    inline bool operator < (const Mat3& a) const {
        return(m_p[0] < a[0] && m_p[1] < a[1] && m_p[2] < a[2]);
    };

    inline bool operator >= (const Mat3& a) const {
        return(m_p[0] >= a[0] && m_p[1] >= a[1] && m_p[2] >= a[2]);
    };

    inline Mat3& init(Vec3<T> x, Vec3<T> y, Vec3<T> z) {
        m_p[0] = x;
        m_p[1] = y;
        m_p[2] = z;
        return (*this);
    };

    inline Mat3 transpose() const {
        return Mat3(
            Vec3<T>(m_p[0][0], m_p[1][0], m_p[2][0]),
            Vec3<T>(m_p[0][1], m_p[1][1], m_p[2][1]),
            Vec3<T>(m_p[0][2], m_p[1][2], m_p[2][2]));
    };

    inline Mat3 identity() const {
        Mat3 result;
        result[0][0] = 1;
        result[1][1] = 1;
        result[2][2] = 1;
        return result;
    };
    

protected:
    Vec3<Vec3<T>> m_p = Vec3<Vec3<T>>(
        Vec3<T>(),
        Vec3<T>(),
        Vec3<T>());
};


template <class T>
inline Mat3<T> operator * (const T& s, const Mat3<T>& P) {
    return (P * s);
}

template <class T>
std::ostream& operator<< (std::ostream& output, const Mat3<T>& v) {
    output << v[0] << " " << v[1] << " " << v[2];
    return output;
}

template <class T>
std::istream& operator>> (std::istream& input, Mat3<T>& v) {
    input >> v[0] >> v[1] >> v[2];
    return input;
}

typedef Mat3<float> Mat3f;
typedef Mat3<double> Mat3d;
typedef Mat3<int> Mat3i;
