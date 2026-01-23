#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <k4a/k4a.h>

#include "kinect/coordinateTransformer.hpp"
#include "kinect/rigidTransformSVD.hpp"
#include "apriltag_detector/apriltagDetector.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>

#include <mutex>
#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <deque>
#include <numeric>
#include <optional>
#include <algorithm>
#include <tuple>
#include <stdexcept>

class TagLocalizerNode : public rclcpp::Node
{
public:
  TagLocalizerNode() : Node("tag_localizer_node")
  {
    // ---------- Topics and basic parameters ----------
    this->declare_parameter<std::string>("raw_calib_topic", "kinect/raw_calibration");
    this->declare_parameter<std::string>("color_topic", "color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "depth/aligned_depth_to_color");
    this->declare_parameter<std::string>("raw_depth_topic", "depth/image_raw");

    // Use raw depth for SVD (more accurate coordinate transformation)
    this->declare_parameter<bool>("use_raw_depth_for_svd", true);

    this->declare_parameter<std::string>("object_pose_topic", "apriltag/object_pose");
    this->declare_parameter<std::string>("object_pose_pnp_topic", "apriltag/object_pose_pnp");
    this->declare_parameter<std::string>("object_pose_svd_topic", "apriltag/object_pose_svd");
    this->declare_parameter<std::string>("object_pose_compare_topic", "apriltag/object_pose_compare");
    this->declare_parameter<std::string>("camera_frame_id", "camera_color_optical_frame");

    // AprilTag detection parameters
    this->declare_parameter<double>("quad_decimate", 1.0);
    this->declare_parameter<double>("quad_sigma", 0.25);
    this->declare_parameter<int>("nthreads", 5);
    this->declare_parameter<bool>("refine_edges", true);

    // Depth validation parameters
    this->declare_parameter<int>("depth_min_mm", 200);
    this->declare_parameter<int>("depth_max_mm", 2000);

    // Depth sampling (robust median window)
    this->declare_parameter<int>("depth_window_radius", 3);
    this->declare_parameter<int>("depth_min_valid_count", 10);
    this->declare_parameter<int>("depth_outlier_thresh_mm", 10);

    // CAD mapping parameters
    this->declare_parameter<std::string>("cad_yaml_path", "config/tag_cad_points.yaml");

    // Point quality filtering parameters
    this->declare_parameter<double>("min_decision_margin", 60.0);
    this->declare_parameter<double>("depth_variance_threshold_mm", 50.0);
    this->declare_parameter<int>("min_tag_coverage", 2);

    // Dynamic tag quality weighting (kept; not required for center-only depth, but retained)
    this->declare_parameter<double>("tag_quality_weight_min", 0.4);
    this->declare_parameter<double>("tag_quality_weight_max", 2.0);
    this->declare_parameter<double>("corner_weight_factor", 0.8);

    // ---------- NEW: Center-depth translation refinement ----------
    this->declare_parameter<bool>("use_depth_center_translation", true);

    // ---------- NEW: Live benchmark toggle (PnP vs SVD) ----------
    this->declare_parameter<bool>("benchmark_pnp_vs_svd", false);

    // EMA for center 3D point in camera frame (time-domain filtering)
    // alpha close to 1.0 -> less smoothing; close to 0.0 -> more smoothing
    this->declare_parameter<double>("ema_alpha_center", 0.25);

    // Reject sudden depth-point jumps (meters gate expressed in mm)
    this->declare_parameter<double>("center_jump_gate_mm", 30.0);

    // Warmup frames: accept measurement directly for first N updates per tag
    this->declare_parameter<int>("center_filter_warmup", 5);

    // Rotation smoothing parameters
    this->declare_parameter<double>("ema_alpha_rot", 0.35);
    this->declare_parameter<int>("rotation_filter_warmup", 5);

    // Optional sanity gate: if depth-based translation differs too much from PnP translation, fallback to PnP
    this->declare_parameter<double>("depth_vs_pnp_gate_mm", 30.0);

    // ---------- NEW: SVD depth offset compensation ----------
    // Compensate for systematic depth bias in ToF sensor (positive = sensor reads too far)
    this->declare_parameter<double>("svd_depth_offset_mm", 22.5);

    // ---------- Read parameters ----------
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();

    depth_min_mm_ = this->get_parameter("depth_min_mm").as_int();
    depth_max_mm_ = this->get_parameter("depth_max_mm").as_int();

    depth_window_radius_ = this->get_parameter("depth_window_radius").as_int();
    depth_min_valid_count_ = this->get_parameter("depth_min_valid_count").as_int();
    depth_outlier_thresh_mm_ = this->get_parameter("depth_outlier_thresh_mm").as_int();

    min_decision_margin_ = this->get_parameter("min_decision_margin").as_double();
    depth_variance_threshold_mm_ = this->get_parameter("depth_variance_threshold_mm").as_double();
    min_tag_coverage_ = this->get_parameter("min_tag_coverage").as_int();

    tag_quality_weight_min_ = this->get_parameter("tag_quality_weight_min").as_double();
    tag_quality_weight_max_ = this->get_parameter("tag_quality_weight_max").as_double();
    corner_weight_factor_ = this->get_parameter("corner_weight_factor").as_double();

    const double quad_decimate = this->get_parameter("quad_decimate").as_double();
    const double quad_sigma = this->get_parameter("quad_sigma").as_double();
    const int nthreads = this->get_parameter("nthreads").as_int();
    const bool refine_edges = this->get_parameter("refine_edges").as_bool();

    // Kinect configuration
    depth_mode_ = K4A_DEPTH_MODE_NFOV_UNBINNED;
    color_res_  = K4A_COLOR_RESOLUTION_1080P;

    // ---------- Initialize AprilTag Detector ----------
    detector_ = std::make_unique<AprilTagDetector>(quad_decimate, quad_sigma, nthreads, refine_edges);
    RCLCPP_INFO(get_logger(), "AprilTag Detector initialized (quad_decimate=%.1f, quad_sigma=%.1f)",
                quad_decimate, quad_sigma);

    // ---------- Publishers ----------
    pub_object_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        this->get_parameter("object_pose_topic").as_string(), 10);

    // NEW: Separate PnP and SVD pose publishers
    pub_object_pose_pnp_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        this->get_parameter("object_pose_pnp_topic").as_string(), 10);

    pub_object_pose_svd_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        this->get_parameter("object_pose_svd_topic").as_string(), 10);

    // NEW: Comparison/diagnostics publisher
    pub_object_pose_compare_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        this->get_parameter("object_pose_compare_topic").as_string(), 10);

    pub_marked_ = image_transport::create_publisher(this, "apriltag/image_marked");

    // ---------- Load CAD mapping ----------
    loadCadPoints();

    // ---------- Raw calibration subscription (latched QoS) ----------
    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local();

    sub_raw_calib_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        this->get_parameter("raw_calib_topic").as_string(),
        qos_latched,
        std::bind(&TagLocalizerNode::rawCalibCb, this, std::placeholders::_1));

    // ---------- ExactTime synchronization: color + aligned depth + raw depth ----------
    {
      auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

      const std::string color_topic = this->get_parameter("color_topic").as_string();
      const std::string aligned_topic = this->get_parameter("depth_topic").as_string();
      const std::string raw_depth_topic = this->get_parameter("raw_depth_topic").as_string();
      use_raw_depth_for_svd_ = this->get_parameter("use_raw_depth_for_svd").as_bool();

      mf_color_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, color_topic, qos);

      mf_aligned_depth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, aligned_topic, qos);

      mf_raw_depth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, raw_depth_topic, qos);

      const int queue_size = 30;
      sync_exact_3_ = std::make_shared<message_filters::TimeSynchronizer<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
          *mf_color_, *mf_aligned_depth_, *mf_raw_depth_, queue_size);

      sync_exact_3_->registerCallback(std::bind(&TagLocalizerNode::onSyncedImages3, this,
                                                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

      RCLCPP_INFO(get_logger(), "ExactTime 3-way sync enabled: color='%s', aligned='%s', raw='%s', queue=%d",
                  color_topic.c_str(), aligned_topic.c_str(), raw_depth_topic.c_str(), queue_size);
      RCLCPP_INFO(get_logger(), "use_raw_depth_for_svd=%s", use_raw_depth_for_svd_ ? "true" : "false");
    }

    // ---------- Initialize stats ----------
    stats_accepted_poses_ = 0;
    pose_interval_buffer_size_ = 20;

    RCLCPP_INFO(get_logger(), "TagLocalizerNode initialized (ExactTime sync + robust depth sampling).");
    RCLCPP_INFO(get_logger(), "Depth window: radius=%d (size=%dx%d), min_valid=%d, outlier_thresh=%d mm",
                depth_window_radius_, 2 * depth_window_radius_ + 1, 2 * depth_window_radius_ + 1,
                depth_min_valid_count_, depth_outlier_thresh_mm_);
    RCLCPP_INFO(get_logger(), "Point quality filtering: decision_margin>=%.1f, depth_variance<=%.1f mm, min_tags>=%d",
                min_decision_margin_, depth_variance_threshold_mm_, min_tag_coverage_);
    RCLCPP_INFO(get_logger(), "Center-depth translation refinement: use=%s, ema_alpha=%.3f, jump_gate=%.1f mm, warmup=%ld, depth_vs_pnp_gate=%.1f mm",
                this->get_parameter("use_depth_center_translation").as_bool() ? "true" : "false",
                this->get_parameter("ema_alpha_center").as_double(),
                this->get_parameter("center_jump_gate_mm").as_double(),
                this->get_parameter("center_filter_warmup").as_int(),
                this->get_parameter("depth_vs_pnp_gate_mm").as_double());
  }

private:
  // -------------------------
  // Raw calibration callback
  // -------------------------
  void rawCalibCb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transformer_) return;

    k4a_calibration_t calib;
    k4a_result_t res = k4a_calibration_get_from_raw(
        reinterpret_cast<char *>(const_cast<uint8_t *>(msg->data.data())),
        msg->data.size(),
        depth_mode_,
        color_res_,
        &calib);

    if (res == K4A_RESULT_SUCCEEDED)
    {
      transformer_ = std::make_unique<ct::CoordinateTransformer>(calib);

      // Extract color camera intrinsics from K4A calibration
      camera_fx_ = calib.color_camera_calibration.intrinsics.parameters.param.fx;
      camera_fy_ = calib.color_camera_calibration.intrinsics.parameters.param.fy;
      camera_cx_ = calib.color_camera_calibration.intrinsics.parameters.param.cx;
      camera_cy_ = calib.color_camera_calibration.intrinsics.parameters.param.cy;

      // Extract distortion coefficients (Brown-Conrady model)
      dist_k1_ = calib.color_camera_calibration.intrinsics.parameters.param.k1;
      dist_k2_ = calib.color_camera_calibration.intrinsics.parameters.param.k2;
      dist_p1_ = calib.color_camera_calibration.intrinsics.parameters.param.p1;
      dist_p2_ = calib.color_camera_calibration.intrinsics.parameters.param.p2;
      dist_k3_ = calib.color_camera_calibration.intrinsics.parameters.param.k3;
      dist_k4_ = calib.color_camera_calibration.intrinsics.parameters.param.k4;
      dist_k5_ = calib.color_camera_calibration.intrinsics.parameters.param.k5;
      dist_k6_ = calib.color_camera_calibration.intrinsics.parameters.param.k6;

      camera_intrinsics_valid_ = true;

      RCLCPP_INFO(get_logger(), "SDK Transformer initialized from Raw Calibration.");
      RCLCPP_INFO(get_logger(), "Color Camera Intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                  camera_fx_, camera_fy_, camera_cx_, camera_cy_);
      RCLCPP_INFO(get_logger(), "Distortion: k1=%.6f, k2=%.6f, p1=%.6f, p2=%.6f, k3=%.6f",
                  dist_k1_, dist_k2_, dist_p1_, dist_p2_, dist_k3_);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse K4A Raw Calibration!");
    }
  }

  // -------------------------
  // Load CAD points from YAML
  // -------------------------
  void loadCadPoints()
  {
    const std::string package_name = "kinect";
    const std::string pkg_share = ament_index_cpp::get_package_share_directory(package_name);
    const std::string rel_path  = this->get_parameter("cad_yaml_path").as_string();
    const std::string yaml_path = pkg_share + "/" + rel_path;

    RCLCPP_INFO(get_logger(), "Loading CAD tag points from: %s", yaml_path.c_str());

    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node flat = root["flat_points"];

    if (!flat || !flat.IsSequence())
    {
      throw std::runtime_error("YAML: 'flat_points' not found or not a sequence");
    }

    int max_index = -1;
    for (const auto &node : flat)
    {
      const int idx = node["index"].as<int>();
      if (idx > max_index) max_index = idx;
    }
    if (max_index < 0)
    {
      throw std::runtime_error("YAML: no valid indices in flat_points");
    }

    cad_points_object_.assign(static_cast<size_t>(max_index + 1), Eigen::Vector3d::Zero());

    for (const auto &node : flat)
    {
      const int idx = node["index"].as<int>();
      auto pos = node["position"];
      if (!pos || !pos.IsSequence() || pos.size() != 3)
      {
        throw std::runtime_error("YAML: 'position' for index " + std::to_string(idx) + " is invalid");
      }

      const double x = pos[0].as<double>();
      const double y = pos[1].as<double>();
      const double z = pos[2].as<double>();

      if (idx < 0 || idx >= static_cast<int>(cad_points_object_.size()))
      {
        throw std::runtime_error("YAML: index out of range in flat_points");
      }

      // Convert from mm to meters
      cad_points_object_[static_cast<size_t>(idx)] = Eigen::Vector3d(x * 0.001, y * 0.001, z * 0.001);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu CAD reference points from YAML.", cad_points_object_.size());
  }

  // -------------------------
  // ExactTime synced callback (3-way: color + aligned_depth + raw_depth)
  // -------------------------
  void onSyncedImages3(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr &aligned_depth_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr &raw_depth_msg)
  {
    ct::CoordinateTransformer *transformer_raw = nullptr;
    int depth_min_mm = 0;
    int depth_max_mm = 0;
    std::string camera_frame_id_local;
    bool use_raw_depth = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!transformer_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                             "Waiting for Raw Calibration (transformer not ready).");
        return;
      }

      transformer_raw = transformer_.get();
      depth_min_mm = depth_min_mm_;
      depth_max_mm = depth_max_mm_;
      camera_frame_id_local = camera_frame_id_;
      use_raw_depth = use_raw_depth_for_svd_;
    }

    processImages(color_msg, aligned_depth_msg, raw_depth_msg, transformer_raw,
                  depth_min_mm, depth_max_mm, camera_frame_id_local, use_raw_depth);
  }

  // -------------------------
  // Helper: compute median without modifying input vector
  // -------------------------
  static uint16_t medianNoModify(std::vector<uint16_t> vals)
  {
    if (vals.empty()) return 0;
    std::sort(vals.begin(), vals.end());
    const size_t n = vals.size();
    const size_t mid = n / 2;
    if (n % 2 == 1) return vals[mid];
    return static_cast<uint16_t>((static_cast<uint32_t>(vals[mid - 1]) + static_cast<uint32_t>(vals[mid])) / 2u);
  }

  // -------------------------
  // Robust depth sampling: window median + min valid + optional trimming
  // Returns: 3D point in meters and depth "std" in mm (named variance in original code)
  // -------------------------
  struct Point3DQuality {
    Eigen::Vector3d point;
    double depth_variance_mm;  // actually std-dev in mm
  };

  struct EMA3D {
    bool initialized = false;
    Eigen::Vector3d y = Eigen::Vector3d::Zero();
    int count = 0;
  };

  struct EMAR {
    bool initialized = false;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    int count = 0;
  };

  std::optional<Point3DQuality> get3DPoint(const cv::Mat &depth_img,
                                           float u, float v,
                                           int min_mm, int max_mm,
                                           ct::CoordinateTransformer *transformer)
  {
    if (!transformer) return std::nullopt;
    if (depth_img.empty()) return std::nullopt;
    if (depth_img.type() != CV_16UC1) return std::nullopt;

    if (!std::isfinite(u) || !std::isfinite(v)) return std::nullopt;

    const int cols = depth_img.cols;
    const int rows = depth_img.rows;
    if (cols <= 0 || rows <= 0) return std::nullopt;

    if (u < 0.0f || u >= static_cast<float>(cols) ||
        v < 0.0f || v >= static_cast<float>(rows))
    {
      return std::nullopt;
    }

    const int uc = std::max(0, std::min(cols - 1, static_cast<int>(std::lround(u))));
    const int vc = std::max(0, std::min(rows - 1, static_cast<int>(std::lround(v))));

    const int r = std::max(0, depth_window_radius_);
    const int u_min = std::max(0, uc - r);
    const int u_max = std::min(cols - 1, uc + r);
    const int v_min = std::max(0, vc - r);
    const int v_max = std::min(rows - 1, vc + r);

    std::vector<uint16_t> samples;
    samples.reserve(static_cast<size_t>((u_max - u_min + 1) * (v_max - v_min + 1)));

    for (int y = v_min; y <= v_max; ++y)
    {
      const uint16_t *row_ptr = depth_img.ptr<uint16_t>(y);
      for (int x = u_min; x <= u_max; ++x)
      {
        const uint16_t d = row_ptr[x];
        if (d == 0) continue;
        if (d < static_cast<uint16_t>(min_mm) || d > static_cast<uint16_t>(max_mm)) continue;
        samples.push_back(d);
      }
    }

    if (static_cast<int>(samples.size()) < std::max(1, depth_min_valid_count_))
    {
      return std::nullopt;
    }

    const uint16_t med = medianNoModify(samples);
    if (med == 0) return std::nullopt;

    uint16_t depth_mm_final = med;

    const int outlier_thresh = std::max(0, depth_outlier_thresh_mm_);
    if (outlier_thresh > 0)
    {
      std::vector<uint16_t> trimmed;
      trimmed.reserve(samples.size());

      const int med_i = static_cast<int>(med);
      for (uint16_t d : samples)
      {
        const int di = static_cast<int>(d);
        if (std::abs(di - med_i) <= outlier_thresh)
        {
          trimmed.push_back(d);
        }
      }

      if (static_cast<int>(trimmed.size()) >= std::max(1, depth_min_valid_count_))
      {
        depth_mm_final = medianNoModify(trimmed);
        if (depth_mm_final == 0) return std::nullopt;
      }
      else
      {
        depth_mm_final = med;
      }
    }

    cv::Point3f pt3d_mm = transformer->ICS2CCS(u, v, depth_mm_final);
    if (!std::isfinite(pt3d_mm.x) || !std::isfinite(pt3d_mm.y) || !std::isfinite(pt3d_mm.z))
    {
      return std::nullopt;
    }

    // Compute depth std-dev in mm from window samples
    double depth_std_mm = 0.0;
    if (samples.size() > 1)
    {
      double mean = 0.0;
      for (uint16_t d : samples) mean += static_cast<double>(d);
      mean /= static_cast<double>(samples.size());

      double var = 0.0;
      for (uint16_t d : samples)
      {
        const double diff = static_cast<double>(d) - mean;
        var += diff * diff;
      }
      depth_std_mm = std::sqrt(var / static_cast<double>(samples.size()));
    }

    Point3DQuality result;
    result.point = Eigen::Vector3d(
        static_cast<double>(pt3d_mm.x) * 0.001,
        static_cast<double>(pt3d_mm.y) * 0.001,
        static_cast<double>(pt3d_mm.z) * 0.001
    );
    result.depth_variance_mm = depth_std_mm;
    return result;
  }

  // -------------------------
  // Helper: Sample median depth from raw depth image at given pixel location
  // -------------------------
  uint16_t sampleMedianDepth(const cv::Mat &raw_depth_img, float px, float py,
                              int min_mm, int max_mm, int window_radius, int min_valid_count)
  {
    const int cols = raw_depth_img.cols;
    const int rows = raw_depth_img.rows;

    const int uc = std::max(0, std::min(cols - 1, static_cast<int>(std::lround(px))));
    const int vc = std::max(0, std::min(rows - 1, static_cast<int>(std::lround(py))));

    const int r = std::max(0, window_radius);
    const int u_min = std::max(0, uc - r);
    const int u_max = std::min(cols - 1, uc + r);
    const int v_min = std::max(0, vc - r);
    const int v_max = std::min(rows - 1, vc + r);

    std::vector<uint16_t> samples;
    samples.reserve(static_cast<size_t>((u_max - u_min + 1) * (v_max - v_min + 1)));

    for (int y = v_min; y <= v_max; ++y)
    {
      const uint16_t *row_ptr = raw_depth_img.ptr<uint16_t>(y);
      for (int x = u_min; x <= u_max; ++x)
      {
        const uint16_t d = row_ptr[x];
        if (d == 0) continue;
        if (d < static_cast<uint16_t>(min_mm) || d > static_cast<uint16_t>(max_mm)) continue;
        samples.push_back(d);
      }
    }

    if (static_cast<int>(samples.size()) < std::max(1, min_valid_count))
    {
      return 0;
    }

    return medianNoModify(samples);
  }

  // -------------------------
  // Get 3D point using RAW depth image + DEPTH→COLOR transformation
  // This uses iterative refinement to correct for parallax errors
  // -------------------------
  std::optional<Point3DQuality> get3DPointRaw(const cv::Mat &raw_depth_img,
                                              float u_color, float v_color,
                                              int min_mm, int max_mm,
                                              ct::CoordinateTransformer *transformer)
  {
    if (!transformer) return std::nullopt;
    if (raw_depth_img.empty()) return std::nullopt;
    if (raw_depth_img.type() != CV_16UC1) return std::nullopt;

    if (!std::isfinite(u_color) || !std::isfinite(v_color)) return std::nullopt;

    const int cols = raw_depth_img.cols;
    const int rows = raw_depth_img.rows;

    // Iterative refinement to correct for parallax
    // Start with center of depth range, then refine using actual sampled depth
    float current_depth_estimate = static_cast<float>((min_mm + max_mm) / 2);
    cv::Point2f depth_pixel;
    uint16_t sampled_depth = 0;

    constexpr int kMaxIterations = 3;
    for (int iter = 0; iter < kMaxIterations; ++iter)
    {
      // Project color pixel to depth image coordinates using current depth estimate
      depth_pixel = transformer->colorPixelToDepthPixel(u_color, v_color, current_depth_estimate);
      if (!std::isfinite(depth_pixel.x) || !std::isfinite(depth_pixel.y))
      {
        return std::nullopt;
      }

      if (depth_pixel.x < 0.0f || depth_pixel.x >= static_cast<float>(cols) ||
          depth_pixel.y < 0.0f || depth_pixel.y >= static_cast<float>(rows))
      {
        return std::nullopt;
      }

      // Sample depth at projected location
      sampled_depth = sampleMedianDepth(raw_depth_img, depth_pixel.x, depth_pixel.y,
                                         min_mm, max_mm, depth_window_radius_, depth_min_valid_count_);
      if (sampled_depth == 0) return std::nullopt;

      // Check for convergence (depth estimate close to sampled depth)
      const float depth_diff = std::abs(current_depth_estimate - static_cast<float>(sampled_depth));
      if (depth_diff < 5.0f)  // Converged within 5mm
      {
        break;
      }

      // Update estimate for next iteration
      current_depth_estimate = static_cast<float>(sampled_depth);
    }

    // Use the refined sampled depth for 3D transformation
    uint16_t depth_mm_final = sampled_depth;

    // Final depth pixel location after refinement
    const int uc = std::max(0, std::min(cols - 1, static_cast<int>(std::lround(depth_pixel.x))));
    const int vc = std::max(0, std::min(rows - 1, static_cast<int>(std::lround(depth_pixel.y))));

    // Apply outlier trimming if enabled - re-sample with final depth pixel location
    const int outlier_thresh = std::max(0, depth_outlier_thresh_mm_);
    std::vector<uint16_t> final_samples;

    if (outlier_thresh > 0)
    {
      const int r = std::max(0, depth_window_radius_);
      const int u_min = std::max(0, uc - r);
      const int u_max = std::min(cols - 1, uc + r);
      const int v_min = std::max(0, vc - r);
      const int v_max = std::min(rows - 1, vc + r);

      final_samples.reserve(static_cast<size_t>((u_max - u_min + 1) * (v_max - v_min + 1)));

      for (int y = v_min; y <= v_max; ++y)
      {
        const uint16_t *row_ptr = raw_depth_img.ptr<uint16_t>(y);
        for (int x = u_min; x <= u_max; ++x)
        {
          const uint16_t d = row_ptr[x];
          if (d == 0) continue;
          if (d < static_cast<uint16_t>(min_mm) || d > static_cast<uint16_t>(max_mm)) continue;
          final_samples.push_back(d);
        }
      }

      // Apply outlier trimming
      std::vector<uint16_t> trimmed;
      trimmed.reserve(final_samples.size());

      const int med_i = static_cast<int>(sampled_depth);
      for (uint16_t d : final_samples)
      {
        const int di = static_cast<int>(d);
        if (std::abs(di - med_i) <= outlier_thresh)
        {
          trimmed.push_back(d);
        }
      }

      if (static_cast<int>(trimmed.size()) >= std::max(1, depth_min_valid_count_))
      {
        depth_mm_final = medianNoModify(trimmed);
        if (depth_mm_final == 0) return std::nullopt;
        final_samples = std::move(trimmed);
      }
    }

    // Use DEPTH→COLOR transformation (SDK's full transformation chain)
    cv::Point3f pt3d_mm = transformer->ICS2CCS_DepthToColor(
        depth_pixel.x, depth_pixel.y, depth_mm_final);

    if (!std::isfinite(pt3d_mm.x) || !std::isfinite(pt3d_mm.y) || !std::isfinite(pt3d_mm.z))
    {
      return std::nullopt;
    }

    // Compute depth std-dev in mm from final samples (for quality assessment)
    double depth_std_mm = 0.0;
    if (final_samples.size() > 1)
    {
      double mean = 0.0;
      for (uint16_t d : final_samples) mean += static_cast<double>(d);
      mean /= static_cast<double>(final_samples.size());

      double var = 0.0;
      for (uint16_t d : final_samples)
      {
        const double diff = static_cast<double>(d) - mean;
        var += diff * diff;
      }
      depth_std_mm = std::sqrt(var / static_cast<double>(final_samples.size()));
    }

    Point3DQuality result;
    result.point = Eigen::Vector3d(
        static_cast<double>(pt3d_mm.x) * 0.001,
        static_cast<double>(pt3d_mm.y) * 0.001,
        static_cast<double>(pt3d_mm.z) * 0.001
    );
    result.depth_variance_mm = depth_std_mm;
    return result;
  }

  Eigen::Matrix3d getFilteredRotation(
      int tag_id,
      const Eigen::Matrix3d& R_meas)
  {
    const double alpha_rot = std::clamp(this->get_parameter("ema_alpha_rot").as_double(), 0.0, 1.0);
    const int warmup = static_cast<int>(std::max<int64_t>(0, this->get_parameter("rotation_filter_warmup").as_int()));

    EMAR& ema = rotation_ema_by_tag_[tag_id];

    if (!ema.initialized) {
      ema.initialized = true;
      ema.R = R_meas;
      ema.count = 1;
      return ema.R;
    }

    if (ema.count < warmup) {
      ema.R = R_meas;
      ema.count++;
      return ema.R;
    }

    // Convert current filtered R to rotation vector
    Eigen::AngleAxisd aa_prev(ema.R);
    Eigen::Vector3d w_prev = aa_prev.axis() * aa_prev.angle();

    // Convert new measurement to rotation vector
    Eigen::AngleAxisd aa_meas(R_meas);
    Eigen::Vector3d w_meas = aa_meas.axis() * aa_meas.angle();

    // EMA on rotation vector
    Eigen::Vector3d w_filtered = (1.0 - alpha_rot) * w_prev + alpha_rot * w_meas;

    // Convert back to rotation matrix
    double angle_filtered = w_filtered.norm();
    if (angle_filtered < 1e-12) {
      ema.R = Eigen::Matrix3d::Identity();
    } else {
      Eigen::Vector3d axis_filtered = w_filtered / angle_filtered;
      ema.R = Eigen::AngleAxisd(angle_filtered, axis_filtered).toRotationMatrix();
    }

    ema.count++;
    return ema.R;
  }

  std::optional<Eigen::Vector3d> getFilteredCenterPointC(
      int tag_id,
      float u,
      float v,
      const cv::Mat& depth_img,
      ct::CoordinateTransformer* transformer,
      int depth_min_mm,
      int depth_max_mm,
      double* out_depth_std_mm)
  {
    if (out_depth_std_mm) *out_depth_std_mm = 0.0;

    auto opt = get3DPoint(depth_img, u, v, depth_min_mm, depth_max_mm, transformer);
    if (!opt.has_value()) return std::nullopt;

    // Gate by depth std-dev threshold
    if (depth_variance_threshold_mm_ > 0.0 &&
        opt->depth_variance_mm > depth_variance_threshold_mm_) {
      return std::nullopt;
    }

    const Eigen::Vector3d p_meas = opt->point;
    if (out_depth_std_mm) *out_depth_std_mm = opt->depth_variance_mm;

    const double alpha = std::clamp(this->get_parameter("ema_alpha_center").as_double(), 0.0, 1.0);
    const double jump_gate_m = this->get_parameter("center_jump_gate_mm").as_double() * 0.001;
    const int warmup = static_cast<int>(std::max<int64_t>(0, this->get_parameter("center_filter_warmup").as_int()));

    EMA3D& ema = center_ema_by_tag_[tag_id];

    if (!ema.initialized) {
      ema.initialized = true;
      ema.y = p_meas;
      ema.count = 1;
      return ema.y;
    }

    // Warmup: follow measurements directly for first N updates
    if (ema.count < warmup) {
      ema.y = p_meas;
      ema.count++;
      return ema.y;
    }

    // Jump gate for robustness
    const double jump = (p_meas - ema.y).norm();
    if (jump > jump_gate_m) {
      // Reject update; keep previous filtered value
      return ema.y;
    }

    // EMA update
    ema.y = (1.0 - alpha) * ema.y + alpha * p_meas;
    ema.count++;
    return ema.y;
  }

  // -------------------------
  // Process synced color + aligned depth + raw depth: AprilTag detection + Pose estimation
  // Multi-Tag Global PnP (2D→3D) + Multi-Tag SVD (3D→3D) computed simultaneously
  // -------------------------
  void processImages(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr &aligned_depth_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr &raw_depth_msg,
                     ct::CoordinateTransformer *transformer,
                     int depth_min_mm,
                     int depth_max_mm,
                     const std::string &camera_frame_id,
                     bool use_raw_depth_for_svd)
  {
    cv_bridge::CvImageConstPtr color_ptr;
    try
    {
      color_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception converting color image: %s", e.what());
      return;
    }

    cv_bridge::CvImageConstPtr aligned_depth_ptr;
    try
    {
      aligned_depth_ptr = cv_bridge::toCvShare(aligned_depth_msg, "16UC1");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception converting aligned depth image: %s", e.what());
      return;
    }

    cv_bridge::CvImageConstPtr raw_depth_ptr;
    try
    {
      raw_depth_ptr = cv_bridge::toCvShare(raw_depth_msg, "16UC1");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception converting raw depth image: %s", e.what());
      return;
    }

    const cv::Mat &color_img = color_ptr->image;
    const cv::Mat &depth_img = aligned_depth_ptr->image;  // For PnP (aligned)
    const cv::Mat &raw_depth_img = raw_depth_ptr->image;  // For SVD (raw)

    if (color_img.empty() || depth_img.empty() || raw_depth_img.empty())
    {
      RCLCPP_WARN(get_logger(), "Empty image received");
      return;
    }

    const rclcpp::Time stamp(color_msg->header.stamp);

    auto detections = detector_->detect(color_img);

    if (pub_marked_.getNumSubscribers() > 0 && !detections.empty())
    {
      cv::Mat marked = detector_->drawDetections(color_img, detections, true, true, 2);
      auto marked_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", marked).toImageMsg();
      marked_msg->header.stamp = stamp;
      marked_msg->header.frame_id = camera_frame_id;
      pub_marked_.publish(marked_msg);
    }

    if (detections.empty())
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "No AprilTags detected.");
      return;
    }

    // Camera matrix and distortion from calibration
    cv::Mat camera_matrix_pnp;
    cv::Mat dist_coeffs_pnp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      camera_matrix_pnp = (cv::Mat_<double>(3, 3) <<
          camera_fx_, 0, camera_cx_,
          0, camera_fy_, camera_cy_,
          0, 0, 1);

      dist_coeffs_pnp = (cv::Mat_<double>(8, 1) <<
          dist_k1_, dist_k2_, dist_p1_, dist_p2_,
          dist_k3_, dist_k4_, dist_k5_, dist_k6_);
    }

    static constexpr std::array<int, 10> kOrder{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static constexpr int kPointsPerTag = 5;  // center + 4 corners

    std::unordered_map<int, const TagDetection*> lookup;
    lookup.reserve(detections.size());
    for (const auto &det : detections)
    {
      lookup[det.id] = &det;
    }

    // ============================================================
    // COLLECT ALL POINTS FROM ALL VALID TAGS FOR GLOBAL ESTIMATION
    // ============================================================

    // Global point collections for Multi-Tag Global PnP
    std::vector<cv::Point2f> global_image_points;
    std::vector<cv::Point3f> global_object_points;

    // Global point collections for Multi-Tag SVD (3D→3D)
    std::vector<Eigen::Vector3d> global_pts_O;  // Object frame (CAD)
    std::vector<Eigen::Vector3d> global_pts_C;  // Camera frame (from depth)
    std::vector<double> global_depth_stds;      // Depth std for quality assessment

    int valid_tag_count = 0;
    std::vector<int> used_tag_ids;

    // ---- First pass: collect valid tags and sort by quality (decision margin) ----
    struct TagQuality {
      int tagId;
      double margin;
      const TagDetection* det;
    };
    std::vector<TagQuality> valid_tags;

    for (size_t idx = 0; idx < kOrder.size(); ++idx)
    {
      const int tagId = kOrder[idx];
      const size_t base_index = static_cast<size_t>(tagId) * kPointsPerTag;

      auto it = lookup.find(tagId);
      if (it == lookup.end()) continue;
      const TagDetection &det = *it->second;

      // Quality gate: decision margin
      if (min_decision_margin_ > 0.0 && det.decisionMargin < min_decision_margin_)
      {
        continue;
      }

      // Check CAD points availability
      if (base_index + 4 >= cad_points_object_.size())
      {
        continue;
      }

      valid_tags.push_back({tagId, det.decisionMargin, &det});
    }

    // Sort by decision margin (descending) and select top 2 for SVD
    std::sort(valid_tags.begin(), valid_tags.end(),
              [](const TagQuality& a, const TagQuality& b) { return a.margin > b.margin; });

    std::unordered_set<int> svd_tag_ids;
    constexpr int kSvdTopTags = 2;
    for (size_t i = 0; i < std::min(static_cast<size_t>(kSvdTopTags), valid_tags.size()); ++i)
    {
      svd_tag_ids.insert(valid_tags[i].tagId);
    }

    if (!valid_tags.empty())
    {
      RCLCPP_INFO(get_logger(), "SVD using top %zu tags: ", svd_tag_ids.size());
      for (int id : svd_tag_ids)
      {
        auto it = std::find_if(valid_tags.begin(), valid_tags.end(),
                               [id](const TagQuality& t) { return t.tagId == id; });
        if (it != valid_tags.end())
        {
          RCLCPP_INFO(get_logger(), "  Tag %d (margin=%.1f)", id, it->margin);
        }
      }
    }

    // ---- Second pass: collect points for PnP (all tags) and SVD (top 2 tags) ----
    for (const auto& tq : valid_tags)
    {
      const int tagId = tq.tagId;
      const TagDetection &det = *tq.det;
      const size_t base_index = static_cast<size_t>(tagId) * kPointsPerTag;
      const bool use_for_svd = (svd_tag_ids.count(tagId) > 0);

      valid_tag_count++;
      used_tag_ids.push_back(tagId);

      // ---- DEBUG: Log point mapping ----
      RCLCPP_INFO(get_logger(), "=== TAG %d (margin=%.1f) %s ===",
                  tagId, det.decisionMargin, use_for_svd ? "[SVD]" : "");
      RCLCPP_INFO(get_logger(), "  Center: det(%.1f, %.1f) -> CAD(%.2f, %.2f, %.2f) mm",
                  det.center.x, det.center.y,
                  cad_points_object_[base_index].x() * 1000.0,
                  cad_points_object_[base_index].y() * 1000.0,
                  cad_points_object_[base_index].z() * 1000.0);
      for (int dbg_i = 0; dbg_i < 4; ++dbg_i) {
          const size_t dbg_idx = base_index + static_cast<size_t>(dbg_i + 1);
          RCLCPP_INFO(get_logger(), "  Corner[%d]: det(%.1f, %.1f) -> CAD(%.2f, %.2f, %.2f) mm",
                      dbg_i, det.corners[dbg_i].x, det.corners[dbg_i].y,
                      cad_points_object_[dbg_idx].x() * 1000.0,
                      cad_points_object_[dbg_idx].y() * 1000.0,
                      cad_points_object_[dbg_idx].z() * 1000.0);
      }

      // ---- Add center point ----
      {
        global_image_points.emplace_back(det.center.x, det.center.y);
        const auto& pt3d = cad_points_object_[base_index];
        global_object_points.emplace_back(
            static_cast<float>(pt3d.x() * 1000.0),
            static_cast<float>(pt3d.y() * 1000.0),
            static_cast<float>(pt3d.z() * 1000.0));

        // Get 3D point from depth for SVD (only for top tags)
        if (use_for_svd)
        {
          std::optional<Point3DQuality> depth_center;
          if (use_raw_depth_for_svd)
          {
            // Use raw depth + DEPTH→COLOR transformation (more accurate)
            depth_center = get3DPointRaw(raw_depth_img, det.center.x, det.center.y,
                                         depth_min_mm, depth_max_mm, transformer);
          }
          else
          {
            // Use aligned depth + COLOR→COLOR transformation (legacy)
            depth_center = get3DPoint(depth_img, det.center.x, det.center.y,
                                      depth_min_mm, depth_max_mm, transformer);
          }

          if (depth_center.has_value())
          {
            global_pts_O.push_back(cad_points_object_[base_index]);
            global_pts_C.push_back(depth_center->point);
            global_depth_stds.push_back(depth_center->depth_variance_mm);
          }
        }
      }

      // ---- Add 4 corners ----
      for (int i = 0; i < 4; ++i)
      {
        const size_t idx_obj = base_index + static_cast<size_t>(i + 1);

        global_image_points.emplace_back(det.corners[i].x, det.corners[i].y);
        const auto& pt3d = cad_points_object_[idx_obj];
        global_object_points.emplace_back(
            static_cast<float>(pt3d.x() * 1000.0),
            static_cast<float>(pt3d.y() * 1000.0),
            static_cast<float>(pt3d.z() * 1000.0));

        // Get 3D point from depth for SVD (only for top tags)
        if (use_for_svd)
        {
          std::optional<Point3DQuality> depth_corner;
          if (use_raw_depth_for_svd)
          {
            // Use raw depth + DEPTH→COLOR transformation (more accurate)
            depth_corner = get3DPointRaw(raw_depth_img, det.corners[i].x, det.corners[i].y,
                                         depth_min_mm, depth_max_mm, transformer);
          }
          else
          {
            // Use aligned depth + COLOR→COLOR transformation (legacy)
            depth_corner = get3DPoint(depth_img, det.corners[i].x, det.corners[i].y,
                                      depth_min_mm, depth_max_mm, transformer);
          }

          if (depth_corner.has_value())
          {
            global_pts_O.push_back(cad_points_object_[idx_obj]);
            global_pts_C.push_back(depth_corner->point);
            global_depth_stds.push_back(depth_corner->depth_variance_mm);
          }
        }
      }
    }

    // Require minimum tag coverage
    if (valid_tag_count < min_tag_coverage_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Insufficient tag coverage: %d < %d",
                           valid_tag_count, min_tag_coverage_);
      return;
    }

    // ============================================================
    // METHOD A: MULTI-TAG GLOBAL PnP (2D→3D)
    // ============================================================

    bool pnp_valid = false;
    Eigen::Matrix3d R_pnp = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_pnp = Eigen::Vector3d::Zero();
    double pnp_reproj_mean_px = 0.0;
    double pnp_inliers_ratio = 0.0;
    int pnp_tags_used = valid_tag_count;

    if (global_image_points.size() >= 4)
    {
      cv::Mat rvec, tvec;
      std::vector<int> inliers;

      bool pnp_success = cv::solvePnPRansac(
          global_object_points, global_image_points, camera_matrix_pnp, dist_coeffs_pnp,
          rvec, tvec, false, 100, 4.0, 0.99, inliers);

      if (pnp_success && inliers.size() >= 4)
      {
        pnp_valid = true;

        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        R_pnp << R_cv.at<double>(0, 0), R_cv.at<double>(0, 1), R_cv.at<double>(0, 2),
                 R_cv.at<double>(1, 0), R_cv.at<double>(1, 1), R_cv.at<double>(1, 2),
                 R_cv.at<double>(2, 0), R_cv.at<double>(2, 1), R_cv.at<double>(2, 2);

        t_pnp = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        t_pnp *= 0.001;  // mm -> m

        // Compute global reprojection error
        std::vector<cv::Point2f> reprojected;
        cv::projectPoints(global_object_points, rvec, tvec, camera_matrix_pnp, dist_coeffs_pnp, reprojected);

        double reproj_sum = 0.0;
        RCLCPP_INFO(get_logger(), "=== PnP Per-Point Reproj Errors ===");
        for (size_t i = 0; i < global_image_points.size(); ++i)
        {
          double dx = global_image_points[i].x - reprojected[i].x;
          double dy = global_image_points[i].y - reprojected[i].y;
          double err = std::sqrt(dx * dx + dy * dy);
          reproj_sum += err;

          // Determine which tag and which point (center or corner 0-3)
          size_t tag_idx = i / 5;
          size_t point_in_tag = i % 5;
          const char* point_name = (point_in_tag == 0) ? "center" :
                                   (point_in_tag == 1) ? "corner0" :
                                   (point_in_tag == 2) ? "corner1" :
                                   (point_in_tag == 3) ? "corner2" : "corner3";
          int tag_id = (tag_idx < used_tag_ids.size()) ? used_tag_ids[tag_idx] : -1;

          RCLCPP_INFO(get_logger(), "  [%zu] Tag%d.%s: det(%.1f,%.1f) -> reproj(%.1f,%.1f) CAD(%.1f,%.1f,%.1f)mm err=%.1f px",
                      i, tag_id, point_name,
                      global_image_points[i].x, global_image_points[i].y,
                      reprojected[i].x, reprojected[i].y,
                      global_object_points[i].x, global_object_points[i].y, global_object_points[i].z,
                      err);
        }
        pnp_reproj_mean_px = reproj_sum / static_cast<double>(global_image_points.size());
        pnp_inliers_ratio = static_cast<double>(inliers.size()) / static_cast<double>(global_image_points.size());
        RCLCPP_INFO(get_logger(), "PnP: mean_reproj=%.2f px, inliers=%zu/%zu (%.1f%%)",
                    pnp_reproj_mean_px, inliers.size(), global_image_points.size(), pnp_inliers_ratio * 100.0);
      }
    }

    // ============================================================
    // METHOD B: MULTI-TAG SVD (3D→3D, Umeyama)
    // ============================================================

    bool svd_valid = false;
    Eigen::Matrix3d R_svd = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_svd = Eigen::Vector3d::Zero();
    double svd_rmse_m = 0.0;
    int svd_pairs_used = static_cast<int>(global_pts_O.size());
    double svd_mean_depth_std_mm = 0.0;

    if (global_pts_O.size() >= 3 && global_pts_C.size() == global_pts_O.size())
    {
      auto svd_result = kinect::RigidTransformSVD::estimate(global_pts_O, global_pts_C);
      if (svd_result.has_value())
      {
        // Reject reflections
        if (svd_result->rotation.determinant() > 0.0)
        {
          svd_valid = true;
          R_svd = svd_result->rotation;
          t_svd = svd_result->translation;
          svd_rmse_m = svd_result->rmse;

          // Compute mean depth std
          if (!global_depth_stds.empty())
          {
            svd_mean_depth_std_mm = std::accumulate(global_depth_stds.begin(), global_depth_stds.end(), 0.0)
                                    / static_cast<double>(global_depth_stds.size());
          }
        }
      }
    }

    // ============================================================
    // PUBLISH RESULTS
    // ============================================================

    stats_accepted_poses_++;

    // Frequency tracking
    double pose_hz = 0.0;
    if (last_pose_time_.nanoseconds() != 0)
    {
      const double interval = (stamp - last_pose_time_).seconds();
      pose_intervals_.push_back(interval);
      while (pose_intervals_.size() > pose_interval_buffer_size_)
      {
        pose_intervals_.pop_front();
      }

      if (!pose_intervals_.empty())
      {
        const double avg_interval =
            std::accumulate(pose_intervals_.begin(), pose_intervals_.end(), 0.0) /
            static_cast<double>(pose_intervals_.size());
        if (avg_interval > 0.0) pose_hz = 1.0 / avg_interval;
      }
    }
    last_pose_time_ = stamp;

    // ---- Topic 1: PnP Pose (/apriltag/object_pose_pnp) ----
    if (pnp_valid)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_pnp_msg;
      pose_pnp_msg.header.stamp = stamp;
      pose_pnp_msg.header.frame_id = camera_frame_id;

      Eigen::Quaterniond q_pnp(R_pnp);
      pose_pnp_msg.pose.pose.position.x = t_pnp.x();
      pose_pnp_msg.pose.pose.position.y = t_pnp.y();
      pose_pnp_msg.pose.pose.position.z = t_pnp.z();
      pose_pnp_msg.pose.pose.orientation.x = q_pnp.x();
      pose_pnp_msg.pose.pose.orientation.y = q_pnp.y();
      pose_pnp_msg.pose.pose.orientation.z = q_pnp.z();
      pose_pnp_msg.pose.pose.orientation.w = q_pnp.w();

      // Diagnostics in covariance (debug purpose):
      // covariance[0]: global reproj mean (px)
      // covariance[1]: RANSAC inliers ratio (0-1)
      // covariance[2]: used tags count
      std::fill(pose_pnp_msg.pose.covariance.begin(), pose_pnp_msg.pose.covariance.end(), 0.0);
      pose_pnp_msg.pose.covariance[0] = pnp_reproj_mean_px;
      pose_pnp_msg.pose.covariance[1] = pnp_inliers_ratio;
      pose_pnp_msg.pose.covariance[2] = static_cast<double>(pnp_tags_used);

      pub_object_pose_pnp_->publish(pose_pnp_msg);

      // Also publish to legacy topic for backward compatibility
      pub_object_pose_->publish(pose_pnp_msg);
    }

    // ---- Topic 2: SVD Pose (/apriltag/object_pose_svd) ----
    if (svd_valid)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_svd_msg;
      pose_svd_msg.header.stamp = stamp;
      pose_svd_msg.header.frame_id = camera_frame_id;

      Eigen::Quaterniond q_svd(R_svd);
      pose_svd_msg.pose.pose.position.x = t_svd.x();
      pose_svd_msg.pose.pose.position.y = t_svd.y();
      pose_svd_msg.pose.pose.position.z = t_svd.z();
      pose_svd_msg.pose.pose.orientation.x = q_svd.x();
      pose_svd_msg.pose.pose.orientation.y = q_svd.y();
      pose_svd_msg.pose.pose.orientation.z = q_svd.z();
      pose_svd_msg.pose.pose.orientation.w = q_svd.w();

      // Diagnostics in covariance (debug purpose):
      // covariance[0]: rmse (m)
      // covariance[1]: used 3D pairs count
      // covariance[2]: mean depth std (mm)
      std::fill(pose_svd_msg.pose.covariance.begin(), pose_svd_msg.pose.covariance.end(), 0.0);
      pose_svd_msg.pose.covariance[0] = svd_rmse_m;
      pose_svd_msg.pose.covariance[1] = static_cast<double>(svd_pairs_used);
      pose_svd_msg.pose.covariance[2] = svd_mean_depth_std_mm;

      pub_object_pose_svd_->publish(pose_svd_msg);
    }

    // ---- Topic 3: Comparison (/apriltag/object_pose_compare) ----
    if (pnp_valid || svd_valid)
    {
      std_msgs::msg::Float64MultiArray compare_msg;

      // data[0] = trans_diff_mm (total)
      // data[1] = rot_diff_deg
      // data[2] = reproj_px (PnP)
      // data[3] = rmse_mm (SVD)
      // data[4] = tags_used_pnp
      // data[5] = pairs_used_svd
      // data[6] = trans_diff_x_mm (SVD - PnP)
      // data[7] = trans_diff_y_mm (SVD - PnP)
      // data[8] = trans_diff_z_mm (SVD - PnP)
      compare_msg.data.resize(9, 0.0);

      if (pnp_valid && svd_valid)
      {
        // Translation difference (per-axis for diagnosis)
        Eigen::Vector3d t_diff = t_svd - t_pnp;  // SVD - PnP
        double trans_diff_m = t_diff.norm();
        compare_msg.data[0] = trans_diff_m * 1000.0;  // mm

        // Rotation difference
        Eigen::Matrix3d R_diff = R_pnp.transpose() * R_svd;
        Eigen::AngleAxisd aa_diff(R_diff);
        compare_msg.data[1] = aa_diff.angle() * 180.0 / M_PI;  // degrees

        // Per-axis translation difference (mm)
        compare_msg.data[6] = t_diff.x() * 1000.0;
        compare_msg.data[7] = t_diff.y() * 1000.0;
        compare_msg.data[8] = t_diff.z() * 1000.0;

        // Detailed logging for diagnosis
        RCLCPP_INFO(get_logger(),
                    "[Diagnosis] PnP t=(%.1f, %.1f, %.1f) mm | SVD t=(%.1f, %.1f, %.1f) mm | diff=(%.1f, %.1f, %.1f) mm",
                    t_pnp.x() * 1000.0, t_pnp.y() * 1000.0, t_pnp.z() * 1000.0,
                    t_svd.x() * 1000.0, t_svd.y() * 1000.0, t_svd.z() * 1000.0,
                    t_diff.x() * 1000.0, t_diff.y() * 1000.0, t_diff.z() * 1000.0);
      }

      compare_msg.data[2] = pnp_valid ? pnp_reproj_mean_px : -1.0;
      compare_msg.data[3] = svd_valid ? (svd_rmse_m * 1000.0) : -1.0;  // mm
      compare_msg.data[4] = static_cast<double>(pnp_tags_used);
      compare_msg.data[5] = static_cast<double>(svd_pairs_used);

      pub_object_pose_compare_->publish(compare_msg);
    }

    // Log summary
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Multi-Tag Pose | PnP: %s (reproj=%.2f px, inliers=%.1f%%, tags=%d) | "
                         "SVD: %s (rmse=%.2f mm, pairs=%d, depth_std=%.1f mm) | %.1f Hz (%zu)",
                         pnp_valid ? "OK" : "FAIL", pnp_reproj_mean_px, pnp_inliers_ratio * 100.0, pnp_tags_used,
                         svd_valid ? "OK" : "FAIL", svd_rmse_m * 1000.0, svd_pairs_used, svd_mean_depth_std_mm,
                         pose_hz, stats_accepted_poses_);

    // Log comparison when both methods succeed
    if (pnp_valid && svd_valid)
    {
      double trans_diff_mm = (t_pnp - t_svd).norm() * 1000.0;
      Eigen::Matrix3d R_diff = R_pnp.transpose() * R_svd;
      Eigen::AngleAxisd aa_diff(R_diff);
      double rot_diff_deg = aa_diff.angle() * 180.0 / M_PI;

      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "[Compare] PnP vs SVD: trans_diff=%.2f mm, rot_diff=%.2f deg",
                           trans_diff_mm, rot_diff_deg);
    }
  }

private:
  // Kinect
  k4a_depth_mode_t depth_mode_;
  k4a_color_resolution_t color_res_;

  // Core components
  std::unique_ptr<ct::CoordinateTransformer> transformer_;
  std::unique_ptr<AprilTagDetector> detector_;
  std::mutex mutex_;

  // Camera intrinsics (from K4A calibration)
  double camera_fx_ = 913.0;
  double camera_fy_ = 913.0;
  double camera_cx_ = 959.0;
  double camera_cy_ = 539.0;
  bool camera_intrinsics_valid_ = false;

  // Distortion coefficients
  double dist_k1_ = 0.0;
  double dist_k2_ = 0.0;
  double dist_p1_ = 0.0;
  double dist_p2_ = 0.0;
  double dist_k3_ = 0.0;
  double dist_k4_ = 0.0;
  double dist_k5_ = 0.0;
  double dist_k6_ = 0.0;

  // Depth validation params
  int depth_min_mm_ = 200;
  int depth_max_mm_ = 2000;

  // Robust depth sampling params
  int depth_window_radius_ = 3;
  int depth_min_valid_count_ = 10;
  int depth_outlier_thresh_mm_ = 0;

  // CAD reference points in object frame {O}
  std::vector<Eigen::Vector3d> cad_points_object_;

  std::string camera_frame_id_;

  // message_filters ExactTime sync (3-way: color + aligned_depth + raw_depth)
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> mf_color_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> mf_aligned_depth_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> mf_raw_depth_;
  std::shared_ptr<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_exact_3_;

  // Use raw depth for SVD
  bool use_raw_depth_for_svd_ = true;

  // Point quality filtering parameters
  double min_decision_margin_ = 30.0;
  double depth_variance_threshold_mm_ = 50.0;
  int min_tag_coverage_ = 3;

  // Dynamic tag quality weighting parameters
  double tag_quality_weight_min_ = 0.5;
  double tag_quality_weight_max_ = 2.0;
  double corner_weight_factor_ = 0.8;

  // EMA state per tag id
  std::unordered_map<int, EMA3D> center_ema_by_tag_;
  std::unordered_map<int, EMAR> rotation_ema_by_tag_;

  // Statistics
  size_t stats_accepted_poses_ = 0;

  // Frequency tracking
  rclcpp::Time last_pose_time_;
  std::deque<double> pose_intervals_;
  size_t pose_interval_buffer_size_ = 20;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_raw_calib_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_object_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_object_pose_pnp_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_object_pose_svd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_object_pose_compare_;
  image_transport::Publisher pub_marked_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}
