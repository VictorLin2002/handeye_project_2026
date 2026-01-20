#pragma once

/* ============================================================================
 * Dual Quaternion Library - Simplified OOP
 *  - Basic encapsulation with constructors
 *  - Keep it simple and practical
 *  - No over-engineering
 * ============================================================================ */

#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

namespace dq
{

/* ============================================================================
 * Quaternion
 * ============================================================================ */
class Quaternion
{
public:
    double w, x, y, z;
    
    /* Constructors */
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    /* Basic operations */
    Quaternion operator*(const Quaternion &q) const;
    Quaternion operator+(const Quaternion &q) const;
    Quaternion operator*(double s) const;
    
    Quaternion conjugate() const;
    double norm() const;
    void normalize();
    
    /* Utility functions */
    static Quaternion fromRotationVector(double rx, double ry, double rz);
    static Quaternion fromRotationMatrix(const Eigen::Matrix3d &R);
    Eigen::Matrix3d toRotationMatrix() const;
    
    void print(int precision = 6) const;
};

/* ============================================================================
 * SE3 - 4x4 transformation matrix
 * ============================================================================ */
class SE3
{
public:
    Eigen::Matrix4d mat;
    
    /* Constructors */
    SE3() : mat(Eigen::Matrix4d::Identity()) {}
    SE3(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    
    /* Accessors */
    double& operator()(int row, int col) { return mat(row, col); }
    const double& operator()(int row, int col) const { return mat(row, col); }
    
    Eigen::Matrix3d rotation() const { return mat.block<3,3>(0,0); }
    Eigen::Vector3d translation() const { return mat.block<3,1>(0,3); }
    
    /* Operations */
    SE3 operator*(const SE3 &T) const { return SE3(mat * T.mat); }
    SE3 inverse() const;
    
    void print(int width = 10, int precision = 6) const;

private:
    SE3(const Eigen::Matrix4d &m) : mat(m) {}
};

/* ============================================================================
 * DualQuaternion
 * ============================================================================ */
class DualQuaternion
{
public:
    Quaternion real;  // Rotation part
    Quaternion dual;  // Translation part
    
    /* Constructors */
    DualQuaternion();
    DualQuaternion(const Quaternion &r, const Quaternion &d);
    
    /* Conversions */
    static DualQuaternion fromTCP(const std::array<double, 6> &tcp);
    static DualQuaternion fromSE3(const SE3 &T);
    SE3 toSE3() const;
    
    /* Operations */
    DualQuaternion operator*(const DualQuaternion &dq) const;
    DualQuaternion inverse() const;
    void normalize();
    
    void print(int precision = 6) const;
};

} /* namespace dq */