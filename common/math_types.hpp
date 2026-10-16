#pragma once

#include <cmath>
#include <glm/glm.hpp>

// ============================================================
// 3D VECTOR
// ============================================================
struct Vec3
{
    double x, y, z;

    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }

    Vec3& operator+=(const Vec3& o)
    {
        x += o.x; y += o.y; z += o.z;
        return *this;
    }

    double Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 Cross(const Vec3& o) const
    {
        return Vec3(
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        );
    }

    double Magnitude() const { return std::sqrt(Dot(*this)); }
};

// ============================================================
// QUATERNION
// ============================================================
struct Quat
{
    double w, x, y, z;

    Quat(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
        : w(w), x(x), y(y), z(z) {}

    Quat operator*(const Quat& q) const
    {
        return Quat(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    Quat Normalized() const
    {
        double magnitude = std::sqrt(w * w + x * x + y * y + z * z);
        if (magnitude < 1e-12)
            return Quat(1.0, 0.0, 0.0, 0.0);
        return Quat(w / magnitude, x / magnitude, y / magnitude, z / magnitude);
    }

    static Quat FromAngularDisplacement(const Vec3& deltaTheta)
    {
        double angle = deltaTheta.Magnitude();
        if (angle < 1e-12)
            return Quat(1.0, 0.0, 0.0, 0.0);

        double halfAngle = angle * 0.5;
        double sinHalf = std::sin(halfAngle);

        return Quat(
            std::cos(halfAngle),
            (deltaTheta.x / angle) * sinHalf,
            (deltaTheta.y / angle) * sinHalf,
            (deltaTheta.z / angle) * sinHalf
        );
    }

    glm::mat4 ToRotationMatrix() const
    {
        glm::mat4 m(1.0f);

        m[0][0] = static_cast<float>(1.0 - 2.0 * (y * y + z * z));
        m[0][1] = static_cast<float>(2.0 * (x * y + w * z));
        m[0][2] = static_cast<float>(2.0 * (x * z - w * y));

        m[1][0] = static_cast<float>(2.0 * (x * y - w * z));
        m[1][1] = static_cast<float>(1.0 - 2.0 * (x * x + z * z));
        m[1][2] = static_cast<float>(2.0 * (y * z + w * x));

        m[2][0] = static_cast<float>(2.0 * (x * z + w * y));
        m[2][1] = static_cast<float>(2.0 * (y * z - w * x));
        m[2][2] = static_cast<float>(1.0 - 2.0 * (x * x + y * y));

        return m;
    }
};
