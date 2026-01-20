#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "handeye_solver/dualQuart.hpp"

// ============================================================
// Math Utilities
// ============================================================
namespace MathUtils {

static inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

static inline double quatDot(const dq::Quaternion& a, const dq::Quaternion& b)
{
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

// Keep dual quaternion sign continuous vs ref
static inline dq::DualQuaternion normalizeSign(dq::DualQuaternion q, const dq::DualQuaternion& ref)
{
    if (quatDot(q.real, ref.real) < 0.0) {
        q.real = dq::Quaternion(-q.real.w, -q.real.x, -q.real.y, -q.real.z);
        q.dual = dq::Quaternion(-q.dual.w, -q.dual.x, -q.dual.y, -q.dual.z);
    }
    return q;
}

static inline double rotationAngleRad(const dq::DualQuaternion& dqv)
{
    double w = std::abs(dqv.real.w);
    w = clamp(w, -1.0, 1.0);
    return 2.0 * std::acos(w);
}

static inline Eigen::Matrix<double, 8, 8> dualQuatL(const dq::DualQuaternion& dqv)
{
    auto qL = [](const dq::Quaternion& q) {
        Eigen::Matrix4d M;
        M << q.w, -q.x, -q.y, -q.z,
             q.x,  q.w, -q.z,  q.y,
             q.y,  q.z,  q.w, -q.x,
             q.z, -q.y,  q.x,  q.w;
        return M;
    };

    Eigen::Matrix<double, 8, 8> M = Eigen::Matrix<double, 8, 8>::Zero();
    M.block<4, 4>(0, 0) = qL(dqv.real);
    M.block<4, 4>(4, 0) = qL(dqv.dual);
    M.block<4, 4>(4, 4) = qL(dqv.real);
    return M;
}

static inline Eigen::Matrix<double, 8, 8> dualQuatR(const dq::DualQuaternion& dqv)
{
    auto qR = [](const dq::Quaternion& q) {
        Eigen::Matrix4d M;
        M << q.w, -q.x, -q.y, -q.z,
             q.x,  q.w,  q.z, -q.y,
             q.y, -q.z,  q.w,  q.x,
             q.z,  q.y, -q.x,  q.w;
        return M;
    };

    Eigen::Matrix<double, 8, 8> M = Eigen::Matrix<double, 8, 8>::Zero();
    M.block<4, 4>(0, 0) = qR(dqv.real);
    M.block<4, 4>(4, 0) = qR(dqv.dual);
    M.block<4, 4>(4, 4) = qR(dqv.real);
    return M;
}

static inline dq::DualQuaternion vecToDQ(const Eigen::Matrix<double, 8, 1>& x)
{
    dq::Quaternion r(x(0), x(1), x(2), x(3));
    dq::Quaternion d(x(4), x(5), x(6), x(7));
    dq::DualQuaternion out(r, d);
    out.normalize();
    return out;
}

static inline Eigen::Vector3d so3Log(const Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd aa(R);
    double angle = aa.angle();
    if (!std::isfinite(angle) || angle < 1e-12) return Eigen::Vector3d::Zero();

    // Eigen may return arbitrary axis when angle ~ 0, guarded above
    return aa.axis() * angle;
}

static inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w)
{
    const double a = w.norm();
    if (a < 1e-12) return Eigen::Matrix3d::Identity();
    return Eigen::AngleAxisd(a, w / a).toRotationMatrix();
}

static inline Eigen::Quaterniond averageQuaternion(const std::vector<Eigen::Quaterniond>& qs)
{
    if (qs.empty()) return Eigen::Quaterniond::Identity();

    Eigen::Vector4d acc(0, 0, 0, 0);
    Eigen::Vector4d ref(qs[0].x(), qs[0].y(), qs[0].z(), qs[0].w());

    for (const auto& q : qs) {
        Eigen::Vector4d v(q.x(), q.y(), q.z(), q.w());
        if (v.dot(ref) < 0.0) v = -v;
        acc += v;
    }

    acc.normalize();
    return Eigen::Quaterniond(acc(3), acc(0), acc(1), acc(2)); // w,x,y,z
}

} // namespace MathUtils

// ============================================================
// Data Structures
// ============================================================
struct Sample
{
    int id;
    dq::DualQuaternion BE; // Base -> EndEffector
    dq::DualQuaternion CO; // Camera -> Object
};

// ============================================================
// CSV Loader
// ============================================================
namespace DataIO {

static std::vector<std::string> splitCSV(const std::string& line)
{
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) out.push_back(token);
    return out;
}

static bool loadSamples(const std::string& path, std::vector<Sample>& samples, bool invert_CO)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string line;
    std::getline(ifs, line); // header

    int idx = 0;
    while (std::getline(ifs, line)) {
        idx++;
        if (line.empty()) continue;

        auto c = splitCSV(line);
        if (c.size() < 15) continue;

        try {
            // Robot: x y z rx ry rz (axis-angle rotvec)
            const double bx = std::stod(c[2]);
            const double by = std::stod(c[3]);
            const double bz = std::stod(c[4]);
            const double brx = std::stod(c[5]);
            const double bry = std::stod(c[6]);
            const double brz = std::stod(c[7]);

            const double angle = std::sqrt(brx*brx + bry*bry + brz*brz);
            dq::Quaternion r_robot(1, 0, 0, 0);
            if (angle > 1e-12) {
                const double s = std::sin(angle * 0.5) / angle;
                r_robot = dq::Quaternion(std::cos(angle * 0.5), brx*s, bry*s, brz*s);
            }

            dq::Quaternion t_robot(0, bx, by, bz);
            dq::DualQuaternion dqBE(r_robot, (t_robot * r_robot) * 0.5);
            dqBE.normalize();

            // Camera: x y z qx qy qz qw
            const double cx = std::stod(c[8]);
            const double cy = std::stod(c[9]);
            const double cz = std::stod(c[10]);
            const double cqx = std::stod(c[11]);
            const double cqy = std::stod(c[12]);
            const double cqz = std::stod(c[13]);
            const double cqw = std::stod(c[14]);

            dq::Quaternion r_cam(cqw, cqx, cqy, cqz); // w,x,y,z
            dq::Quaternion t_cam(0, cx, cy, cz);
            dq::DualQuaternion dqCO(r_cam, (t_cam * r_cam) * 0.5);
            dqCO.normalize();

            if (invert_CO) dqCO = dqCO.inverse();

            samples.push_back({idx, dqBE, dqCO});
        } catch (...) {
            continue;
        }
    }

    return !samples.empty();
}

} // namespace DataIO

// ============================================================
// Filtering
// ============================================================
namespace Filters {

static std::vector<Sample> filterByConsistency(
    const std::vector<Sample>& input,
    double max_diff_deg,
    double min_move_deg)
{
    if (input.empty()) return {};

    std::vector<Sample> out;
    out.push_back(input.front());

    const double max_rad = max_diff_deg * M_PI / 180.0;
    const double min_rad = min_move_deg * M_PI / 180.0;

    std::cout << "\n[Filter] Consistency check: max_diff=" << max_diff_deg
              << " deg, min_move=" << min_move_deg << " deg\n";

    int rejected = 0;
    for (size_t i = 1; i < input.size(); ++i) {
        const auto& prev = out.back();
        const auto& curr = input[i];

        const dq::DualQuaternion rel_robot = prev.BE.inverse() * curr.BE;
        const dq::DualQuaternion rel_cam   = prev.CO.inverse() * curr.CO;

        const double theta_robot = MathUtils::rotationAngleRad(rel_robot);
        if (theta_robot < min_rad) continue;

        const double theta_cam = MathUtils::rotationAngleRad(rel_cam);
        const double diff = std::abs(theta_robot - theta_cam);

        if (diff <= max_rad) {
            out.push_back(curr);
        } else {
            rejected++;
        }
    }

    std::cout << "  kept=" << out.size() << "/" << input.size()
              << " rejected=" << rejected << "\n";
    return out;
}

static void enforceSignContinuity(std::vector<Sample>& samples)
{
    if (samples.size() < 2) return;
    for (size_t i = 1; i < samples.size(); ++i) {
        samples[i].BE = MathUtils::normalizeSign(samples[i].BE, samples[i - 1].BE);
        samples[i].CO = MathUtils::normalizeSign(samples[i].CO, samples[i - 1].CO);
    }
}

} // namespace Filters

// ============================================================
// Solver: Y first (DQ), then refine Z (6DoF) with LM
// ============================================================
namespace Solver {

// 1) Solve Y from AX=XB using DQ linear system
static bool solveY_AXeqXB(const std::vector<Sample>& samples, dq::DualQuaternion& Y_out)
{
    if (samples.size() < 6) return false;

    const size_t N = samples.size() - 1;
    Eigen::MatrixXd M(8 * N, 8);

    for (size_t i = 0; i < N; ++i) {
        dq::DualQuaternion A = samples[i].BE.inverse() * samples[i + 1].BE;
        dq::DualQuaternion B = samples[i].CO.inverse() * samples[i + 1].CO;
        A.normalize();
        B.normalize();
        M.block<8, 8>(static_cast<int>(i * 8), 0) = MathUtils::dualQuatL(A) - MathUtils::dualQuatR(B);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    const Eigen::MatrixXd V = svd.matrixV();

    const Eigen::VectorXd u1 = V.col(6);
    const Eigen::VectorXd u2 = V.col(7);

    const Eigen::Vector4d r1 = u1.head<4>();
    const Eigen::Vector4d r2 = u2.head<4>();

    Eigen::Matrix<double, 8, 1> sol = (r1.norm() > r2.norm()) ? u1 : u2;
    Y_out = MathUtils::vecToDQ(sol);
    return true;
}

// 2) Initial Z from candidates (Z_i = BE_i * Y * CO_i^{-1}) and averaging
static bool estimateZFromCandidates(
    const std::vector<Sample>& samples,
    const dq::DualQuaternion& Y,
    Eigen::Matrix3d& R_BC_init,
    Eigen::Vector3d& t_BC_init)
{
    if (samples.size() < 3) return false;

    std::vector<Eigen::Quaterniond> qs;
    qs.reserve(samples.size());
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();

    for (const auto& s : samples) {
        dq::DualQuaternion Z_i = s.BE * Y * s.CO.inverse();
        Z_i.normalize();

        const dq::SE3 z = Z_i.toSE3();
        qs.emplace_back(Eigen::Quaterniond(z.rotation()));
        t_sum += z.translation();
    }

    const Eigen::Quaterniond q_avg = MathUtils::averageQuaternion(qs);
    R_BC_init = q_avg.toRotationMatrix();
    t_BC_init = t_sum / static_cast<double>(samples.size());
    return true;
}

// 3) Residual report for diagnostics: compare (Z * CO) vs (BE * Y)
static void reportResiduals(
    const std::vector<Sample>& samples,
    const Eigen::Matrix3d& R_BC,
    const Eigen::Vector3d& t_BC,
    const dq::DualQuaternion& Y,
    const std::string& label)
{
    const dq::SE3 y_se3 = Y.toSE3();
    const Eigen::Matrix3d R_Y = y_se3.rotation();
    const Eigen::Vector3d t_Y = y_se3.translation();

    double sum_t_mm = 0.0, max_t_mm = 0.0;
    double sum_r_deg = 0.0, max_r_deg = 0.0;

    for (const auto& s : samples) {
        const dq::SE3 be = s.BE.toSE3();
        const dq::SE3 co = s.CO.toSE3();

        const Eigen::Matrix3d R_BE = be.rotation();
        const Eigen::Vector3d t_BE = be.translation();
        const Eigen::Matrix3d R_CO = co.rotation();
        const Eigen::Vector3d t_CO = co.translation();

        const Eigen::Matrix3d R_left = R_BC * R_CO;
        const Eigen::Vector3d t_left = t_BC + R_BC * t_CO;

        const Eigen::Matrix3d R_right = R_BE * R_Y;
        const Eigen::Vector3d t_right = t_BE + R_BE * t_Y;

        const Eigen::Matrix3d R_err = R_left.transpose() * R_right;
        const Eigen::Vector3d w = MathUtils::so3Log(R_err);
        const double r_deg = w.norm() * 180.0 / M_PI;

        const double t_mm = (t_left - t_right).norm() * 1000.0;

        sum_t_mm += t_mm;
        max_t_mm = std::max(max_t_mm, t_mm);
        sum_r_deg += r_deg;
        max_r_deg = std::max(max_r_deg, r_deg);
    }

    const double mean_t = sum_t_mm / static_cast<double>(samples.size());
    const double mean_r = sum_r_deg / static_cast<double>(samples.size());

    std::cout << "\n===== RESIDUALS: " << label << " =====\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Translation: mean=" << mean_t << " mm  max=" << max_t_mm << " mm\n";
    std::cout << "Rotation:    mean=" << mean_r << " deg max=" << max_r_deg << " deg\n";
}

// 4) LM refine Z in SE(3): optimize [w(3), t(3)] with fixed Y
//
// Residual per sample (6):
//   r_rot = log( (R_BC R_CO)^T (R_BE R_Y) )      [rad]
//   r_t   = (t_BC + R_BC t_CO) - (t_BE + R_BE t_Y)  [m]
// We scale translation to mm so it dominates similarly to your previous practice.
struct ZSE3LMFunctor
{
    using Scalar = double;
    enum { InputsAtCompileTime = Eigen::Dynamic, ValuesAtCompileTime = Eigen::Dynamic };
    using InputType = Eigen::VectorXd;
    using ValueType = Eigen::VectorXd;
    using JacobianType = Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>;

    const std::vector<Sample>& samples;
    const Eigen::Matrix3d R_Y;
    const Eigen::Vector3d t_Y;
    const int m_inputs;
    const int m_values;

    const double trans_scale_mm;   // multiply translation residual (m) by this -> mm
    const double rot_scale;        // multiply rotation residual (rad) by this (typically 1.0)

    ZSE3LMFunctor(const std::vector<Sample>& s,
                  const Eigen::Matrix3d& RY,
                  const Eigen::Vector3d& tY,
                  double trans_scale_mm_in,
                  double rot_scale_in)
        : samples(s),
          R_Y(RY),
          t_Y(tY),
          m_inputs(6),
          m_values(static_cast<int>(s.size()) * 6),
          trans_scale_mm(trans_scale_mm_in),
          rot_scale(rot_scale_in)
    {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }

    int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
    {
        // x = [w_BC(3), t_BC(3)]
        const Eigen::Vector3d w_BC = x.segment<3>(0);
        const Eigen::Vector3d t_BC = x.segment<3>(3);

        const Eigen::Matrix3d R_BC = MathUtils::so3Exp(w_BC);

        for (size_t i = 0; i < samples.size(); ++i) {
            const dq::SE3 be = samples[i].BE.toSE3();
            const dq::SE3 co = samples[i].CO.toSE3();

            const Eigen::Matrix3d R_BE = be.rotation();
            const Eigen::Vector3d t_BE = be.translation();
            const Eigen::Matrix3d R_CO = co.rotation();
            const Eigen::Vector3d t_CO = co.translation();

            // Rotation residual
            // We want: R_BC * R_CO == R_BE * R_Y
            const Eigen::Matrix3d R_left  = R_BC * R_CO;
            const Eigen::Matrix3d R_right = R_BE * R_Y;
            const Eigen::Matrix3d R_err = R_left.transpose() * R_right;
            const Eigen::Vector3d r_rot = MathUtils::so3Log(R_err) * rot_scale; // rad

            // Translation residual
            // We want: t_BC + R_BC t_CO == t_BE + R_BE t_Y
            const Eigen::Vector3d t_left  = t_BC + R_BC * t_CO;
            const Eigen::Vector3d t_right = t_BE + R_BE * t_Y;
            const Eigen::Vector3d r_t_mm = (t_left - t_right) * trans_scale_mm; // mm

            const int base = static_cast<int>(i * 6);
            fvec(base + 0) = r_rot.x();
            fvec(base + 1) = r_rot.y();
            fvec(base + 2) = r_rot.z();
            fvec(base + 3) = r_t_mm.x();
            fvec(base + 4) = r_t_mm.y();
            
            fvec(base + 5) = r_t_mm.z();
        }

        return 0;
    }
};

static bool refineZWithLM_6DoF(
    const std::vector<Sample>& samples,
    const dq::DualQuaternion& Y,
    Eigen::Matrix3d& R_BC_io,
    Eigen::Vector3d& t_BC_io,
    int maxfev,
    double trans_scale_mm = 1000.0,
    double rot_scale = 1.0)
{
    if (samples.size() < 3) return false;

    const dq::SE3 y_se3 = Y.toSE3();
    const Eigen::Matrix3d R_Y = y_se3.rotation();
    const Eigen::Vector3d t_Y = y_se3.translation();

    // Parameterize rotation with rotation-vector (log)
    Eigen::Vector3d w0 = MathUtils::so3Log(R_BC_io);

    Eigen::VectorXd x(6);
    x.segment<3>(0) = w0;
    x.segment<3>(3) = t_BC_io;

    ZSE3LMFunctor functor(samples, R_Y, t_Y, trans_scale_mm, rot_scale);
    Eigen::NumericalDiff<ZSE3LMFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ZSE3LMFunctor>> lm(numDiff);

    lm.parameters.maxfev = maxfev;
    lm.parameters.xtol = 1e-10;
    lm.parameters.ftol = 1e-10;

    const auto status = lm.minimize(x);

    // Update
    const Eigen::Vector3d w_opt = x.segment<3>(0);
    const Eigen::Vector3d t_opt = x.segment<3>(3);

    R_BC_io = MathUtils::so3Exp(w_opt);
    t_BC_io = t_opt;

    // Some Eigen versions do not expose iterations; keep compatible output.
    std::cout << "  [LM-Z] final_norm=" << lm.fvec.norm()
              << " status=" << static_cast<int>(status) << "\n";
    return true;
}

} // namespace Solver

// ============================================================
// Main
// ============================================================
int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: ./handeye_solver <data.csv> [options]\n"
                  << "Options:\n"
                  << "  --consistency-deg D   (default: 1.0)\n"
                  << "  --min-move-deg D      (default: 3.0)\n"
                  << "  --invert-co           invert camera->object\n"
                  << "  --lm-maxfev N         (default: 1200)\n"
                  << "  --lm-trans-scale S    translation scale to mm (default: 1000)\n"
                  << "  --lm-rot-scale S      rotation residual scale (default: 1.0)\n";
        return 1;
    }

    const std::string csv_path = argv[1];

    double consistency_max_diff_deg = 1.0;
    double consistency_min_move_deg = 3.0;
    bool invert_CO = false;

    int lm_maxfev = 1200;
    double lm_trans_scale = 1000.0; // m -> mm
    double lm_rot_scale = 1.0;

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--consistency-deg" && i + 1 < argc) {
            consistency_max_diff_deg = std::stod(argv[++i]);
        } else if (arg == "--min-move-deg" && i + 1 < argc) {
            consistency_min_move_deg = std::stod(argv[++i]);
        } else if (arg == "--invert-co") {
            invert_CO = true;
        } else if (arg == "--lm-maxfev" && i + 1 < argc) {
            lm_maxfev = std::stoi(argv[++i]);
        } else if (arg == "--lm-trans-scale" && i + 1 < argc) {
            lm_trans_scale = std::stod(argv[++i]);
        } else if (arg == "--lm-rot-scale" && i + 1 < argc) {
            lm_rot_scale = std::stod(argv[++i]);
        }
    }

    // 1) Load
    std::vector<Sample> raw_samples;
    if (!DataIO::loadSamples(csv_path, raw_samples, invert_CO)) {
        std::cerr << "[Error] Failed to load CSV: " << csv_path << "\n";
        return 1;
    }
    std::cout << "Loaded " << raw_samples.size() << " samples.\n";

    // 2) Filter
    auto samples = Filters::filterByConsistency(raw_samples, consistency_max_diff_deg, consistency_min_move_deg);
    if (samples.size() < 6) {
        std::cerr << "[Error] Too few valid samples after filtering.\n";
        return 2;
    }
    Filters::enforceSignContinuity(samples);
    std::cout << "Using " << samples.size() << " samples.\n";

    // 3) Solve Y via DQ AX=XB
    dq::DualQuaternion Y;
    if (!Solver::solveY_AXeqXB(samples, Y)) {
        std::cerr << "[Error] Failed to solve Y (AX=XB). Data may be degenerate.\n";
        return 3;
    }

    const dq::SE3 y_se3 = Y.toSE3();
    std::cout << "\n===== SOLVED Y (EndEffector->Object) =====\n";
    std::cout << "t_Y (m): " << y_se3.translation().transpose() << "\n";
    
    const Eigen::Matrix3d R_Y = y_se3.rotation();
    std::cout << "\nR_Y (rotation matrix):\n";
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < 3; ++i) {
        std::cout << R_Y(i,0) << " " << R_Y(i,1) << " " << R_Y(i,2) << "\n";
    }
    
    const Eigen::Quaterniond q_Y(R_Y);
    std::cout << "\nR_Y as quaternion (x y z w): "
              << q_Y.x() << " " << q_Y.y() << " " << q_Y.z() << " " << q_Y.w() << "\n";
    
    std::cout << "\nY as homogeneous matrix (4x4):\n";
    std::cout << R_Y(0,0) << " " << R_Y(0,1) << " " << R_Y(0,2) << " " << y_se3.translation()(0) << "\n";
    std::cout << R_Y(1,0) << " " << R_Y(1,1) << " " << R_Y(1,2) << " " << y_se3.translation()(1) << "\n";
    std::cout << R_Y(2,0) << " " << R_Y(2,1) << " " << R_Y(2,2) << " " << y_se3.translation()(2) << "\n";
    std::cout << "0 0 0 1\n";

    // 4) Z initial from candidates
    Eigen::Matrix3d R_BC = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_BC = Eigen::Vector3d::Zero();

    if (!Solver::estimateZFromCandidates(samples, Y, R_BC, t_BC)) {
        std::cerr << "[Error] Failed to estimate initial Z from candidates.\n";
        return 4;
    }

    std::cout << "\n===== INITIAL Z (Base->Camera) =====\n";
    std::cout << "t_BC_init (m): " << t_BC.transpose() << "\n";
    std::cout << "\n[Homogeneous Matrix - INIT]\n";
    std::cout << R_BC(0,0) << " " << R_BC(0,1) << " " << R_BC(0,2) << " " << t_BC(0) << "\n";
    std::cout << R_BC(1,0) << " " << R_BC(1,1) << " " << R_BC(1,2) << " " << t_BC(1) << "\n";
    std::cout << R_BC(2,0) << " " << R_BC(2,1) << " " << R_BC(2,2) << " " << t_BC(2) << "\n";
    std::cout << "0 0 0 1\n";

    Solver::reportResiduals(samples, R_BC, t_BC, Y, "BEFORE LM (Z 6DoF)");

    // 5) Refine Z in 6DoF with LM (R_BC and t_BC)
    std::cout << "\n[LM] Refining Z in SE(3) (6DoF)...\n";
    if (!Solver::refineZWithLM_6DoF(samples, Y, R_BC, t_BC, lm_maxfev, lm_trans_scale, lm_rot_scale)) {
        std::cerr << "[Warn] LM refinement failed. Keeping initial Z.\n";
    }

    Solver::reportResiduals(samples, R_BC, t_BC, Y, "AFTER LM (Z 6DoF)");

    // 6) Output final T_BC
    const Eigen::Quaterniond q_final(R_BC);

    std::cout << "\n===== FINAL RESULT: T_Base_Camera =====\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Translation (x y z): " << t_BC.x() << " " << t_BC.y() << " " << t_BC.z() << "\n";
    std::cout << "Rotation (x y z w):  " << q_final.x() << " " << q_final.y() << " " << q_final.z() << " " << q_final.w() << "\n";

    std::cout << "\n[Homogeneous Matrix]\n";
    std::cout << R_BC(0,0) << " " << R_BC(0,1) << " " << R_BC(0,2) << " " << t_BC(0) << "\n";
    std::cout << R_BC(1,0) << " " << R_BC(1,1) << " " << R_BC(1,2) << " " << t_BC(1) << "\n";
    std::cout << R_BC(2,0) << " " << R_BC(2,1) << " " << R_BC(2,2) << " " << t_BC(2) << "\n";
    std::cout << "0 0 0 1\n";

    return 0;
}
