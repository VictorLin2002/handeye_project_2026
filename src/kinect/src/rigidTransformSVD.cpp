#include "kinect/rigidTransformSVD.hpp"
#include <cmath>
#include <limits>

namespace kinect {

std::optional<RigidTransformResult> RigidTransformSVD::estimate(
    const std::vector<Eigen::Vector3d>& points_object,
    const std::vector<Eigen::Vector3d>& points_camera)
{
  const size_t num_points = points_object.size();

  // Validate inputs
  if (num_points < 3 || points_camera.size() != num_points)
  {
    return std::nullopt;
  }

  // Compute centroids
  const Eigen::Vector3d centroid_object = computeCentroid(points_object);
  const Eigen::Vector3d centroid_camera = computeCentroid(points_camera);

  // Center the points
  Eigen::MatrixXd centered_object(3, num_points);
  Eigen::MatrixXd centered_camera(3, num_points);

  for (size_t i = 0; i < num_points; ++i)
  {
    centered_object.col(i) = points_object[i] - centroid_object;
    centered_camera.col(i) = points_camera[i] - centroid_camera;
  }

  // Compute cross-covariance matrix
  Eigen::Matrix3d H = centered_object * centered_camera.transpose();

  // SVD decomposition
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Compute rotation
  Eigen::Matrix3d rotation = V * U.transpose();

  // Ensure proper rotation (determinant = +1)
  if (rotation.determinant() < 0.0)
  {
    // Reflection detected - flip the sign of the third column of V
    V.col(2) *= -1.0;
    rotation = V * U.transpose();
  }

  // Compute translation
  Eigen::Vector3d translation = centroid_camera - rotation * centroid_object;

  // Compute RMSE
  const double rmse = computeRMSE(points_object, points_camera, rotation, translation);

  return RigidTransformResult(rotation, translation, rmse);
}

Eigen::Vector3d RigidTransformSVD::computeCentroid(const std::vector<Eigen::Vector3d>& points)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& pt : points)
  {
    centroid += pt;
  }
  return centroid / static_cast<double>(points.size());
}

Eigen::Vector3d RigidTransformSVD::computeWeightedCentroid(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<double>& weights)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double total_weight = 0.0;

  for (size_t i = 0; i < points.size(); ++i)
  {
    centroid += weights[i] * points[i];
    total_weight += weights[i];
  }

  return centroid / total_weight;
}

double RigidTransformSVD::computeRMSE(
    const std::vector<Eigen::Vector3d>& points_object,
    const std::vector<Eigen::Vector3d>& points_camera,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation)
{
  double sum_sq_error = 0.0;
  const size_t num_points = points_object.size();

  for (size_t i = 0; i < num_points; ++i)
  {
    const Eigen::Vector3d transformed = rotation * points_object[i] + translation;
    const double error = (transformed - points_camera[i]).norm();
    sum_sq_error += error * error;
  }

  return std::sqrt(sum_sq_error / static_cast<double>(num_points));
}

double RigidTransformSVD::computeWeightedRMSE(
    const std::vector<Eigen::Vector3d>& points_object,
    const std::vector<Eigen::Vector3d>& points_camera,
    const std::vector<double>& weights,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation)
{
  double weighted_sum_sq_error = 0.0;
  double total_weight = 0.0;
  const size_t num_points = points_object.size();

  for (size_t i = 0; i < num_points; ++i)
  {
    const Eigen::Vector3d transformed = rotation * points_object[i] + translation;
    const double error = (transformed - points_camera[i]).norm();
    weighted_sum_sq_error += weights[i] * error * error;
    total_weight += weights[i];
  }

  return std::sqrt(weighted_sum_sq_error / total_weight);
}

std::optional<RigidTransformResult> RigidTransformSVD::estimateWeighted(
    const std::vector<Eigen::Vector3d>& points_object,
    const std::vector<Eigen::Vector3d>& points_camera,
    const std::vector<double>& weights)
{
  const size_t num_points = points_object.size();

  // Validate inputs
  if (num_points < 3 || points_camera.size() != num_points || weights.size() != num_points)
  {
    return std::nullopt;
  }

  // Validate weights (must be positive)
  for (const double w : weights)
  {
    if (w <= 0.0 || !std::isfinite(w))
    {
      return std::nullopt;
    }
  }

  // Compute weighted centroids
  const Eigen::Vector3d centroid_object = computeWeightedCentroid(points_object, weights);
  const Eigen::Vector3d centroid_camera = computeWeightedCentroid(points_camera, weights);

  // Center the points and apply sqrt of weights
  Eigen::MatrixXd centered_object(3, num_points);
  Eigen::MatrixXd centered_camera(3, num_points);

  for (size_t i = 0; i < num_points; ++i)
  {
    const double sqrt_weight = std::sqrt(weights[i]);
    centered_object.col(i) = sqrt_weight * (points_object[i] - centroid_object);
    centered_camera.col(i) = sqrt_weight * (points_camera[i] - centroid_camera);
  }

  // Compute weighted cross-covariance matrix
  Eigen::Matrix3d H = centered_object * centered_camera.transpose();

  // SVD decomposition
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Compute rotation
  Eigen::Matrix3d rotation = V * U.transpose();

  // Ensure proper rotation (determinant = +1)
  if (rotation.determinant() < 0.0)
  {
    // Reflection detected - flip the sign of the third column of V
    V.col(2) *= -1.0;
    rotation = V * U.transpose();
  }

  // Compute translation
  Eigen::Vector3d translation = centroid_camera - rotation * centroid_object;

  // Compute weighted RMSE
  const double rmse = computeWeightedRMSE(points_object, points_camera, weights, rotation, translation);

  return RigidTransformResult(rotation, translation, rmse);
}

} // namespace kinect
