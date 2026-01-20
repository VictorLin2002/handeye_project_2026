#include "handeye_solver/dualQuart.hpp"

namespace dq
{

/* ============================================================================
 * Quaternion implementation
 * ============================================================================ */

Quaternion Quaternion::operator*(const Quaternion &q) const
{
    return Quaternion(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w + y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
}

Quaternion Quaternion::operator+(const Quaternion &q) const
{
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

Quaternion Quaternion::operator*(double s) const
{
    return Quaternion(w*s, x*s, y*s, z*s);
}

Quaternion Quaternion::conjugate() const
{
    return Quaternion(w, -x, -y, -z);
}

double Quaternion::norm() const
{
    return std::sqrt(w*w + x*x + y*y + z*z);
}

void Quaternion::normalize()
{
    double n = norm();
    if (n > 1e-12)
    {
        w /= n; x /= n; y /= n; z /= n;
    }
}

Quaternion Quaternion::fromRotationVector(double rx, double ry, double rz)
{
    double theta = std::sqrt(rx*rx + ry*ry + rz*rz);
    
    if (theta < 1e-12)
        return Quaternion(1.0, 0.5*rx, 0.5*ry, 0.5*rz);
    
    double half_theta = 0.5 * theta;
    double s = std::sin(half_theta) / theta;
    
    return Quaternion(std::cos(half_theta), s*rx, s*ry, s*rz);
}

Quaternion Quaternion::fromRotationMatrix(const Eigen::Matrix3d &R)
{
    Eigen::Quaterniond q(R);
    q.normalize();
    return Quaternion(q.w(), q.x(), q.y(), q.z());
}

Eigen::Matrix3d Quaternion::toRotationMatrix() const
{
    Eigen::Matrix3d R;
    
    R << 1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y),
             2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x),
             2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y);
    
    return R;
}

void Quaternion::print(int precision) const
{
    std::cout << std::setprecision(precision) << std::fixed
              << "[" << w << ", " << x << ", " << y << ", " << z << "]" << std::endl;
}

/* ============================================================================
 * SE3 implementation
 * ============================================================================ */

SE3::SE3(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
    mat.setIdentity();
    mat.block<3,3>(0,0) = R;
    mat.block<3,1>(0,3) = t;
}

SE3 SE3::inverse() const
{
    Eigen::Matrix3d R = rotation();
    Eigen::Vector3d t = translation();
    
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv * t;
    
    return SE3(R_inv, t_inv);
}

void SE3::print(int width, int precision) const
{
    std::cout << std::setprecision(precision) << std::fixed;
    
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::setw(width) << mat(i, j) << ' ';
        }
        std::cout << '\n';
    }
}

/* ============================================================================
 * DualQuaternion implementation
 * ============================================================================ */

DualQuaternion::DualQuaternion()
    : real(1, 0, 0, 0), dual(0, 0, 0, 0) {}

DualQuaternion::DualQuaternion(const Quaternion &r, const Quaternion &d)
    : real(r), dual(d) {}

DualQuaternion DualQuaternion::fromTCP(const std::array<double, 6> &tcp)
{
    // Rotation part
    Quaternion r = Quaternion::fromRotationVector(tcp[3], tcp[4], tcp[5]);
    r.normalize();
    
    // Translation part: dual = 0.5 * t_quat * real
    Quaternion t(0, tcp[0], tcp[1], tcp[2]);
    Quaternion d = (t * r) * 0.5;
    
    return DualQuaternion(r, d);
}

DualQuaternion DualQuaternion::fromSE3(const SE3 &T)
{
    Quaternion r = Quaternion::fromRotationMatrix(T.rotation());
    r.normalize();
    
    Eigen::Vector3d t = T.translation();
    Quaternion t_quat(0, t.x(), t.y(), t.z());
    Quaternion d = (t_quat * r) * 0.5;
    
    return DualQuaternion(r, d);
}

SE3 DualQuaternion::toSE3() const
{
    Eigen::Matrix3d R = real.toRotationMatrix();
    
    // t = 2 * dual * real_conjugate
    Quaternion t_quat = (dual * real.conjugate()) * 2.0;
    Eigen::Vector3d t(t_quat.x, t_quat.y, t_quat.z);
    
    return SE3(R, t);
}

DualQuaternion DualQuaternion::operator*(const DualQuaternion &dq) const
{
    Quaternion new_real = real * dq.real;
    Quaternion new_dual = (real * dq.dual) + (dual * dq.real);
    
    DualQuaternion result(new_real, new_dual);
    result.normalize();
    
    return result;
}

DualQuaternion DualQuaternion::inverse() const
{
    Quaternion r_conj = real.conjugate();
    Quaternion d_inv = (r_conj * dual * r_conj) * (-1.0);
    
    return DualQuaternion(r_conj, d_inv);
}

void DualQuaternion::normalize()
{
    real.normalize();
    
    // Enforce orthogonality: <real, dual> = 0
    double dot = real.w*dual.w + real.x*dual.x + real.y*dual.y + real.z*dual.z;
    dual = dual + (real * (-dot));
    
    // Ensure positive w
    if (real.w < 0)
    {
        real = real * (-1.0);
        dual = dual * (-1.0);
    }
}

void DualQuaternion::print(int precision) const
{
    std::cout << std::setprecision(precision) << std::fixed;
    std::cout << "Real: [" << real.w << ", " << real.x << ", " << real.y << ", " << real.z << "]" << std::endl;
    std::cout << "Dual: [" << dual.w << ", " << dual.x << ", " << dual.y << ", " << dual.z << "]" << std::endl;
}

} /* namespace dq */