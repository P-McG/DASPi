// DASPi-rotation.h
/*
 * The following is used for compile-time rotation of vectors. 
 */
 
#pragma once

#include <array>
#include <cstddef>

namespace DASPi {

struct Matrix3dData {
    using Row = std::array<double, 3>;

    std::array<Row, 3> m{};

    constexpr Row& operator[](std::size_t i)
    {
        return m[i];
    }

    constexpr const Row& operator[](std::size_t i) const
    {
        return m[i];
    }
};

struct Vector3dData {
    std::array<double, 3> v{};

    constexpr double& operator[](std::size_t i)
    {
        return v[i];
    }

    constexpr const double& operator[](std::size_t i) const
    {
        return v[i];
    }
};

constexpr Matrix3dData MatrixMultiply(const Matrix3dData& a,
                                      const Matrix3dData& b)
{
    Matrix3dData out{};

    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            for (std::size_t k = 0; k < 3; ++k) {
                out[r][c] += a[r][k] * b[k][c];
            }
        }
    }

    return out;
}

constexpr Vector3dData MatrixVectorMultiply(const Matrix3dData& a,
                                            const Vector3dData& v)
{
    Vector3dData out{};

    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t k = 0; k < 3; ++k) {
            out[r] += a[r][k] * v[k];
        }
    }

    return out;
}

constexpr Matrix3dData IdentityMatrix3d()
{
    return Matrix3dData{{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}},
    }}};
}

constexpr Matrix3dData Translation2D(double tx, double ty)
{
    return Matrix3dData{{{
        {{1.0, 0.0, tx }},
        {{0.0, 1.0, ty }},
        {{0.0, 0.0, 1.0}}
    }}};
}

constexpr Matrix3dData RotationXFromSinCos(double c, double s)
{
    return Matrix3dData{{{
        {{1.0, 0.0, 0.0}},
        {{0.0,   c,  -s}},
        {{0.0,   s,   c}}
    }}};
}

constexpr Matrix3dData RotationYFromSinCos(double c, double s)
{
    return Matrix3dData{{{
        {{  c, 0.0,   s}},
        {{0.0, 1.0, 0.0}},
        {{ -s, 0.0,   c}}
    }}};
}

constexpr Matrix3dData RotationZFromSinCos(double c, double s)
{
    return Matrix3dData{{{
        {{  c,  -s, 0.0}},
        {{  s,   c, 0.0}},
        {{0.0, 0.0, 1.0}}
    }}};
}

constexpr Matrix3dData RotationZAroundPointFromSinCos(double centerX,
                                                      double centerY,
                                                      double c,
                                                      double s)
{
    return MatrixMultiply(
        MatrixMultiply(
            Translation2D(centerX, centerY),
            RotationZFromSinCos(c, s)
        ),
        Translation2D(-centerX, -centerY)
    );
}

constexpr Matrix3dData RotationYXFromSinCos(double cy,
                                            double sy,
                                            double cx,
                                            double sx)
{
    return MatrixMultiply(
        RotationYFromSinCos(cy, sy),
        RotationXFromSinCos(cx, sx)
    );
}

} // namespace DASPi
