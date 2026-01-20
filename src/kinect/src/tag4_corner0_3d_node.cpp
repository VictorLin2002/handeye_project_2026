#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <rcl/time.h>

#include <k4a/k4a.h>
#include "kinect/coordinateTransformer.hpp"
#include "kinect/apriltagDetector.hpp"

#include <mutex>
#include <deque>
#include <cstdint>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <string>
#include <optional>

class Tag4Corner0To3D : public rclcpp::Node
{
public:
  Tag4Corner0To3D() : Node("tag4_corner0_to_3d")
  {
    // -------------------------
    // Parameters
    // -------------------------
    this->declare_parameter<std::string>("raw_calib_topic", "kinect/raw_calibration");
    // Match main pipeline defaults
    this->declare_parameter<std::string>("depth_topic", "/depth/aligned_depth_to_color");
    this->declare_parameter<std::string>("color_topic", "/color/image_raw");
    this->declare_parameter<std::string>("out_topic0", "/apriltag/tag4_corner0_3d");
    this->declare_parameter<std::string>("out_topic1", "/apriltag/tag4_corner1_3d");
    this->declare_parameter<std::string>("out_topic2", "/apriltag/tag4_corner2_3d");
    this->declare_parameter<std::string>("out_topic3", "/apriltag/tag4_corner3_3d");
    this->declare_parameter<std::string>("camera_frame_id", "camera_color_optical_frame");

    this->declare_parameter<int>("target_tag_id", 10);  // 確認這裡是 10

    // AprilTag detection parameters
    this->declare_parameter<double>("quad_decimate", 1.0);
    this->declare_parameter<double>("quad_sigma", 0.0);
    this->declare_parameter<int>("nthreads", 2);
    this->declare_parameter<bool>("refine_edges", true);

    // Depth sampling
    this->declare_parameter<int>("window_size", 5);            // odd: 1,3,5,7...
    this->declare_parameter<int>("depth_min_mm", 200);
    this->declare_parameter<int>("depth_max_mm", 4000);

    // Time pairing
    this->declare_parameter<int>("depth_buffer_size", 90);     // ~3 seconds at 30fps
    this->declare_parameter<double>("max_time_diff", 0.05);    // seconds
    this->declare_parameter<bool>("strict_sync", true);        // drop if mismatch

    // Reprojection error gating
    this->declare_parameter<bool>("enable_reproj_gate", true);
    this->declare_parameter<double>("reproj_error_threshold_px", 500.0);  // 暫時放寬到 500px 診斷

    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    target_tag_id_   = this->get_parameter("target_tag_id").as_int();

    window_size_ = this->get_parameter("window_size").as_int();
    if (window_size_ < 1) window_size_ = 1;
    if ((window_size_ % 2) == 0) window_size_ += 1;

    depth_min_mm_ = this->get_parameter("depth_min_mm").as_int();
    depth_max_mm_ = this->get_parameter("depth_max_mm").as_int();

    const int64_t depth_buf_param = this->get_parameter("depth_buffer_size").as_int();
    depth_buffer_size_ = static_cast<size_t>(std::max<int64_t>(1, depth_buf_param));
    max_time_diff_ = this->get_parameter("max_time_diff").as_double();
    strict_sync_ = this->get_parameter("strict_sync").as_bool();

    enable_reproj_gate_ = this->get_parameter("enable_reproj_gate").as_bool();
    reproj_error_threshold_px_ = this->get_parameter("reproj_error_threshold_px").as_double();

    // Initialize AprilTag detector
    const double quad_decimate = this->get_parameter("quad_decimate").as_double();
    const double quad_sigma = this->get_parameter("quad_sigma").as_double();
    const int nthreads = this->get_parameter("nthreads").as_int();
    const bool refine_edges = this->get_parameter("refine_edges").as_bool();

    detector_ = std::make_unique<AprilTagDetector>(quad_decimate, quad_sigma, nthreads, refine_edges);

    // Kinect configuration should match your Azure Kinect driver
    depth_mode_ = K4A_DEPTH_MODE_NFOV_UNBINNED;
    color_res_  = K4A_COLOR_RESOLUTION_1080P;

    // -------------------------
    // Subscribers
    // -------------------------
    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local();

    sub_raw_calib_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      this->get_parameter("raw_calib_topic").as_string(),
      qos_latched,
      std::bind(&Tag4Corner0To3D::rawCalibCb, this, std::placeholders::_1)
    );

    sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("depth_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&Tag4Corner0To3D::depthCb, this, std::placeholders::_1)
    );

    sub_color_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("color_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&Tag4Corner0To3D::colorCb, this, std::placeholders::_1)
    );

    // -------------------------
    // Publisher
    // -------------------------
    pub_corner3d_[0] = this->create_publisher<geometry_msgs::msg::PointStamped>(
      this->get_parameter("out_topic0").as_string(), 10
    );
    pub_corner3d_[1] = this->create_publisher<geometry_msgs::msg::PointStamped>(
      this->get_parameter("out_topic1").as_string(), 10
    );
    pub_corner3d_[2] = this->create_publisher<geometry_msgs::msg::PointStamped>(
      this->get_parameter("out_topic2").as_string(), 10
    );
    pub_corner3d_[3] = this->create_publisher<geometry_msgs::msg::PointStamped>(
      this->get_parameter("out_topic3").as_string(), 10
    );
    pub_center3d_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/apriltag/tag4_center_3d", 10
    );  // NEW: publish tag center

    RCLCPP_INFO(get_logger(), "Tag4Corner0To3D initialized (publishing 4 corners, standalone mode - no apriltag node needed).");
    RCLCPP_INFO(get_logger(), "target_tag_id=%d, window_size=%d, depth=[%d,%d]mm",
                target_tag_id_, window_size_, depth_min_mm_, depth_max_mm_);
    RCLCPP_INFO(get_logger(), "depth_buffer_size=%zu, max_time_diff=%.3fs, strict_sync=%s",
                depth_buffer_size_, max_time_diff_, strict_sync_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "AprilTag: quad_decimate=%.1f, quad_sigma=%.1f, nthreads=%d, refine_edges=%s",
                quad_decimate, quad_sigma, nthreads, refine_edges ? "true" : "false");
    RCLCPP_INFO(get_logger(), "Reprojection Gate: enabled=%s, threshold=%.2f px",
                enable_reproj_gate_ ? "true" : "false", reproj_error_threshold_px_);
  }

private:
  struct DepthFrame
  {
    int64_t stamp_ns = 0;
    double stamp_sec = 0.0;
    cv::Mat img; // CV_16UC1
  };

  // -------------------------
  // Raw calibration callback
  // -------------------------
  void rawCalibCb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transformer_) return;

    k4a_calibration_t calib;
    k4a_result_t res = k4a_calibration_get_from_raw(
      reinterpret_cast<char*>(const_cast<uint8_t*>(msg->data.data())),
      msg->data.size(),
      depth_mode_,
      color_res_,
      &calib
    );

    if (res == K4A_RESULT_SUCCEEDED)
    {
      transformer_ = std::make_unique<ct::CoordinateTransformer>(calib);
      
      // Extract color camera intrinsics from K4A calibration
      camera_fx_ = calib.color_camera_calibration.intrinsics.parameters.param.fx;
      camera_fy_ = calib.color_camera_calibration.intrinsics.parameters.param.fy;
      camera_cx_ = calib.color_camera_calibration.intrinsics.parameters.param.cx;
      camera_cy_ = calib.color_camera_calibration.intrinsics.parameters.param.cy;
      camera_intrinsics_valid_ = true;
      
      RCLCPP_INFO(get_logger(), "K4A CoordinateTransformer ready (from raw calibration).");
      RCLCPP_INFO(get_logger(), "Camera Intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                  camera_fx_, camera_fy_, camera_cx_, camera_cy_);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse K4A raw calibration.");
    }
  }

  // -------------------------
  // Depth callback -> buffer
  // -------------------------
  void depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg, "16UC1");

      DepthFrame f;
      const rclcpp::Time depth_stamp(msg->header.stamp);
      const int64_t ns = depth_stamp.nanoseconds();
      f.stamp_ns = ns;
      f.stamp_sec = static_cast<double>(ns) * 1e-9;
      f.img = ptr->image.clone();

      depth_buf_.push_back(std::move(f));
      while (depth_buf_.size() > depth_buffer_size_)
      {
        depth_buf_.pop_front();
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "depthCb exception: %s", e.what());
    }
  }

  // -------------------------
  // Color callback -> AprilTag detection + PnP solving
  // -------------------------
  void colorCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transformer_ || !detector_ || !camera_intrinsics_valid_) return;

    try
    {
      // Convert ROS image to OpenCV
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat color_img = cv_ptr->image;

      if (color_img.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Empty color image received.");
        return;
      }

      // Get timestamp
      const rclcpp::Time color_stamp(msg->header.stamp);
      const double color_stamp_sec = static_cast<double>(color_stamp.nanoseconds()) * 1e-9;

      // Detect AprilTags
      std::vector<TagDetection> detections = detector_->detect(color_img);

      // Find target tag
      const TagDetection* tag = nullptr;
      for (const auto& det : detections)
      {
        if (det.id == target_tag_id_)
        {
          tag = &det;
          break;
        }
      }

      if (!tag)
      {
        // Target tag not detected
        return;
      }

      // Use AprilTag center directly (simpler than PnP)
      processCenterFromTag(tag, color_stamp_sec, color_stamp);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "colorCb exception: %s", e.what());
    }
  }

  // NEW: Process tag center directly from AprilTag detection
  void processCenterFromTag(const TagDetection* tag, double det_stamp_sec, const rclcpp::Time& color_stamp)
  {
    // Tag center in image space (2D pixel)
    double center_u = tag->center.x;
    double center_v = tag->center.y;

    cv::Mat depth_img;
    double time_diff = 0.0;
    int64_t depth_stamp_ns = 0;
    double depth_stamp_sec = 0.0;
    if (!getClosestDepth(det_stamp_sec, depth_img, time_diff, depth_stamp_ns, depth_stamp_sec))
      return;

    if (time_diff > max_time_diff_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "Tag%d time mismatch: |det-depth|=%.4f s > %.4f s",
                           target_tag_id_, time_diff, max_time_diff_);
      if (strict_sync_) return;
    }

    // Get median depth at center
    uint16_t depth_mm = 0;
    if (!depthMedianAt(depth_img, static_cast<int>(center_u), static_cast<int>(center_v), window_size_, depth_mm))
      return;

    // Transform pixel + depth to 3D camera frame
    cv::Point3f center_3d = transformer_->ICS2CCS(center_u, center_v, depth_mm);
    if (!std::isfinite(center_3d.x) || !std::isfinite(center_3d.y) || !std::isfinite(center_3d.z))
      return;

    // Publish center point
    geometry_msgs::msg::PointStamped center_out;
    center_out.header.stamp = color_stamp;
    center_out.header.frame_id = camera_frame_id_;
    center_out.point.x = static_cast<double>(center_3d.x) * 0.001;  // mm to m
    center_out.point.y = static_cast<double>(center_3d.y) * 0.001;
    center_out.point.z = static_cast<double>(center_3d.z) * 0.001;
    pub_center3d_->publish(center_out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Tag%d: center pixel=[%.1f,%.1f] depth=%u mm -> 3D=(%.3f, %.3f, %.3f) m",
      target_tag_id_, center_u, center_v, depth_mm,
      center_out.point.x, center_out.point.y, center_out.point.z
    );
  }

  bool getClosestDepth(double target_sec,
                       cv::Mat &out_img,
                       double &out_diff_sec,
                       int64_t &out_stamp_ns,
                       double &out_stamp_sec) const
  {
    if (depth_buf_.empty()) return false;

    size_t best_i = 0;
    double best_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < depth_buf_.size(); ++i)
    {
      const double d = std::abs(depth_buf_[i].stamp_sec - target_sec);
      if (d < best_diff)
      {
        best_diff = d;
        best_i = i;
      }
    }

    const auto &entry = depth_buf_[best_i];
    out_img = entry.img;
    out_diff_sec = best_diff;
    out_stamp_ns = entry.stamp_ns;
    out_stamp_sec = entry.stamp_sec;
    return !out_img.empty();
  }

  // -------------------------
  // Depth median with gating
  // -------------------------
  bool depthMedianAt(const cv::Mat& depth16u, int u, int v, int win, uint16_t& depth_mm_out) const
  {
    if (depth16u.empty()) return false;
    if (depth16u.type() != CV_16UC1) return false;

    const int h = depth16u.rows;
    const int w = depth16u.cols;
    if (u < 0 || u >= w || v < 0 || v >= h) return false;

    const int r = win / 2;
    const int u1 = std::max(0, u - r);
    const int u2 = std::min(w - 1, u + r);
    const int v1 = std::max(0, v - r);
    const int v2 = std::min(h - 1, v + r);

    std::vector<uint16_t> vals;
    vals.reserve(static_cast<size_t>((u2 - u1 + 1) * (v2 - v1 + 1)));

    for (int y = v1; y <= v2; ++y)
    {
      const uint16_t* row = depth16u.ptr<uint16_t>(y);
      for (int x = u1; x <= u2; ++x)
      {
        const uint16_t d = row[x];
        if (d == 0) continue;
        if (d < static_cast<uint16_t>(depth_min_mm_) || d > static_cast<uint16_t>(depth_max_mm_)) continue;
        vals.push_back(d);
      }
    }

    if (vals.empty()) return false;

    const size_t mid = vals.size() / 2;
    std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
    depth_mm_out = vals[mid];
    return true;
  }

  void processCorner(int corner_idx, double det_stamp_sec, double u_in, double v_in)
  {
    cv::Mat depth_img;
    double time_diff = 0.0;
    int64_t depth_stamp_ns = 0;
    double depth_stamp_sec = 0.0;
    if (!getClosestDepth(det_stamp_sec, depth_img, time_diff, depth_stamp_ns, depth_stamp_sec)) return;

    if (time_diff > max_time_diff_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "Time mismatch: |det-depth|=%.4f s > %.4f s (strict=%s).",
                           time_diff, max_time_diff_, strict_sync_ ? "true" : "false");
      if (strict_sync_) return;
    }

    const int u = static_cast<int>(std::lround(u_in));
    const int v = static_cast<int>(std::lround(v_in));

    uint16_t depth_mm = 0;
    if (!depthMedianAt(depth_img, u, v, window_size_, depth_mm)) return;

    cv::Point3f p3 = transformer_->ICS2CCS(u_in, v_in, depth_mm);
    if (!std::isfinite(p3.x) || !std::isfinite(p3.y) || !std::isfinite(p3.z)) return;

    rclcpp::Time depth_stamp(depth_stamp_ns, RCL_SYSTEM_TIME);

    geometry_msgs::msg::PointStamped out;
    out.header.stamp = depth_stamp;
    out.header.frame_id = camera_frame_id_;
    out.point.x = static_cast<double>(p3.x) * 0.001;
    out.point.y = static_cast<double>(p3.y) * 0.001;
    out.point.z = static_cast<double>(p3.z) * 0.001;

    if (corner_idx >= 0 && corner_idx < 4 && pub_corner3d_[corner_idx])
    {
      pub_corner3d_[corner_idx]->publish(out);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Tag%d corner%d: uv=(%d,%d) depth=%u mm | det-time=%.6f depth-time=%.6f dt=%.4f s | C=(%.4f, %.4f, %.4f) m",
      target_tag_id_, corner_idx, u, v, depth_mm,
      det_stamp_sec, depth_stamp_sec, time_diff,
      out.point.x, out.point.y, out.point.z
    );
  }

private:
  mutable std::mutex mutex_;

  k4a_depth_mode_t depth_mode_;
  k4a_color_resolution_t color_res_;
  std::unique_ptr<ct::CoordinateTransformer> transformer_;
  std::unique_ptr<AprilTagDetector> detector_;

  // Camera intrinsics (extracted from K4A calibration)
  double camera_fx_ = 913.0;
  double camera_fy_ = 913.0;
  double camera_cx_ = 959.0;
  double camera_cy_ = 539.0;
  bool camera_intrinsics_valid_ = false;

  // Tag4 CAD coordinates (object frame) - 50mm tag
  // Corner order: 0=bottom-left, 1=bottom-right, 2=top-right, 3=top-left (CCW from corner 0)
  // Center is at origin, Z=0 (flat tag)
  static constexpr double TAG_SIZE_MM = 40.0;
  static constexpr double HALF_SIZE_MM = TAG_SIZE_MM / 2.0;
  cv::Point3f tag_corners_cad_[4] = {
    cv::Point3f(-HALF_SIZE_MM, -HALF_SIZE_MM, 0.0f),  // corner 0
    cv::Point3f(+HALF_SIZE_MM, -HALF_SIZE_MM, 0.0f),  // corner 1
    cv::Point3f(+HALF_SIZE_MM, +HALF_SIZE_MM, 0.0f),  // corner 2
    cv::Point3f(-HALF_SIZE_MM, +HALF_SIZE_MM, 0.0f)   // corner 3
  };
  cv::Point3f tag_center_cad_ = cv::Point3f(0.0f, 0.0f, 0.0f);

  // Depth buffer
  std::deque<DepthFrame> depth_buf_;
  size_t depth_buffer_size_ = 90;
  double max_time_diff_ = 0.05;
  bool strict_sync_ = true;

  // Sampling
  int target_tag_id_ = 4;
  int window_size_ = 5;
  int depth_min_mm_ = 200;
  int depth_max_mm_ = 4000;

  std::string camera_frame_id_;

  // Reprojection error gating
  bool enable_reproj_gate_ = true;
  double reproj_error_threshold_px_ = 2.0;
  bool last_corners_valid_ = false;
  geometry_msgs::msg::PointStamped last_valid_corners_[4];

  // Statistics for reprojection error filtering
  uint64_t total_pnp_frames_ = 0;
  uint64_t rejected_frames_ = 0;
  uint64_t accepted_frames_ = 0;
  double sum_accepted_errors_ = 0.0;  // For computing average error

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_raw_calib_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_corner3d_[4];
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_center3d_;  // NEW
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tag4Corner0To3D>());
  rclcpp::shutdown();
  return 0;
}
