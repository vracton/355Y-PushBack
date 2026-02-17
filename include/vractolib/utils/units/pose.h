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
                double newX = cosTheta * x + sinTheta * y;
                double newY = cosTheta * y - sinTheta * x;
                return Pose(newX, newY, theta);
            }

            static double distance(const Pose &p1, const Pose &p2) {
                return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
            }

            //TODO: figure out how to use this properly
            static double angleBetween(const Pose &from, const Pose &to) {
                return std::atan2(to.y - from.y, to.x - from.x);
            }
    };
}

#endif