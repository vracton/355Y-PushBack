#include <string>

#ifndef VECTOR3D_H
#define VECTOR3D_H

namespace vunits {
    template<typename T> class Vector3D {
        public:
            T x, y, z;

            Vector3D() : x(0.0), y(0.0), z(0.0) {}
            Vector3D(T x, T y, T z) : x(x), y(y), z(z) {}

            Vector3D operator=(const Vector3D<T> &v1) {
                if (this != &v1) {
                    x = v1.x;
                    y = v1.y;
                    z = v1.z;
                }
                return *this;
            }

            Vector3D operator+(const Vector3D<T> &v1) const {
                return Vector3D<T>(x+v1.x, y+v1.y, z+v1.z);
            }

            Vector3D operator-(const Vector3D<T> &v1) const {
                return Vector3D<T>(x-v1.x, y-v1.y, z-v1.z);
            }

            Vector3D operator*(const T &s) const {
                return Vector3D<T>(s*x, s*y, s*z);
            }

            Vector3D operator/(const T &s) const {
                return Vector3D<T>(x/s, y/s, z/s);
            }

            Vector3D operator*(const Vector3D<T> &v1) const {
                return Vector3D<T>(x*v1.x, y*v1.y, z*v1.z);
            }

            Vector3D& operator+=(const Vector3D<T> &v1) {
                x += v1.x;
                y += v1.y;
                z += v1.z;
                return *this;
            }

            Vector3D& operator*=(const T &s) {
                x *= s;
                y *= s;
                z *= s;
                return *this;
            }
    };
}

#endif