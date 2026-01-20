#ifndef KINECT_RIGID_TRANSFORM_SVD_HPP
#define KINECT_RIGID_TRANSFORM_SVD_HPP

#include <Eigen/Dense>
#include <vector>
#include <optional>

namespace kinect {

/**
 * @brief Result of rigid transformation estimation
 */
struct RigidTransformResult
{
  Eigen::Matrix3d rotation;       ///< 3x3 rotation matrix
  Eigen::Vector3d translation;    ///< 3D translation vector
  double rmse;                     ///< Root mean square error in meters

  RigidTransformResult()
    : rotation(Eigen::Matrix3d::Identity()),
      translation(Eigen::Vector3d::Zero()),
      rmse(std::numeric_limits<double>::quiet_NaN())
  {}

  RigidTransformResult(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double error)
    : rotation(R), translation(t), rmse(error)
  {}
};

/**
 * @brief Utility class for rigid transformation estimation using SVD
 *
 * This class provides methods to estimate the optimal rigid transformation
 * (rotation + translation) that aligns two sets of 3D point correspondences
 * using Singular Value Decomposition (SVD).
 *
 * Reference: "Least-Squares Fitting of Two 3-D Point Sets" by Arun et al.
 */
class RigidTransformSVD
{
public:
  /**
   * @brief Estimate rigid transformation from object frame to camera frame
   *
   * Solves for rotation R and translation t such that:
   *   points_camera[i] ≈ R * points_object[i] + t
   *
   * @param points_object Reference points in object coordinate frame {O}
   * @param points_camera Measured points in camera coordinate frame {C}
   * @return Result containing rotation, translation, and RMSE. Returns nullopt if estimation fails.
   *
   * @note Requires at least 3 non-collinear point correspondences
   * @note Input vectors must have the same size
   */
  static std::optional<RigidTransformResult> estimate(
      const std::vector<Eigen::Vector3d>& points_object,
      const std::vector<Eigen::Vector3d>& points_camera);

  /**
   * @brief Estimate weighted rigid transformation from object frame to camera frame
   *
   * Solves for rotation R and translation t such that:
   *   points_camera[i] ≈ R * points_object[i] + t
   * using weighted least squares where each point has an associated weight.
   *
   * @param points_object Reference points in object coordinate frame {O}
   * @param points_camera Measured points in camera coordinate frame {C}
   * @param weights Weight for each point correspondence (higher = more important)
   * @return Result containing rotation, translation, and weighted RMSE. Returns nullopt if estimation fails.
   *
   * @note Requires at least 3 non-collinear point correspondences
   * @note Input vectors must have the same size
   * @note Weights are automatically normalized to sum to 1.0
   */
  static std::optional<RigidTransformResult> estimateWeighted(
      const std::vector<Eigen::Vector3d>& points_object,
      const std::vector<Eigen::Vector3d>& points_camera,
      const std::vector<double>& weights);

    /**
     * @brief Compute RMSE between transformed object points and camera points
     */
    static double computeRMSE(
      const std::vector<Eigen::Vector3d>& points_object,
      const std::vector<Eigen::Vector3d>& points_camera,
      const Eigen::Matrix3d& rotation,
      const Eigen::Vector3d& translation);

private:
  /**
   * @brief Compute centroid of a set of 3D points
   */
  static Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& points);

  /**
   * @brief Compute weighted centroid of a set of 3D points
   */
  static Eigen::Vector3d computeWeightedCentroid(
      const std::vector<Eigen::Vector3d>& points,
      const std::vector<double>& weights);

  /**
   * @brief Compute weighted RMSE between transformed object points and camera points
   */
  static double computeWeightedRMSE(
      const std::vector<Eigen::Vector3d>& points_object,
      const std::vector<Eigen::Vector3d>& points_camera,
      const std::vector<double>& weights,
      const Eigen::Matrix3d& rotation,
      const Eigen::Vector3d& translation);
};

} // namespace kinect

#endif // KINECT_RIGID_TRANSFORM_SVD_HPP
