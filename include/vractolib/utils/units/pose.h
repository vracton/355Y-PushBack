#include "vractolib/utils/units/vector3d.h"

#ifndef POSE_H
#define POSE_H

namespace vunits {
    class Pose: public Vector3D<double> {
        public:
            using Vector3D<double>::Vector3D;

            Pose() = default;
            Pose(double x, double y, double theta) : Vector3D(x, y, theta) {};
            Pose(const Vector3D<double> &v) : Vector3D(v.x, v.y, v.z) {};
            
            double &theta = this->z;

            Pose operator=(const Pose &p1) {
                if (this != &p1) {
                    x = p1.x;
                    y = p1.y;
                    theta = p1.theta;
                }
                return *this;
            }

            Pose rotatedBy(const double &angle) const {
                double cosTheta = std::cos(angle);
                double sinTheta = std::sin(angle);
                double newX = cosTheta * x - sinTheta * y;
                double newY = sinTheta * x + cosTheta * y;
                return Pose(newX, newY, theta);
            }
    };
}

#endif