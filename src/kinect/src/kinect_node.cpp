#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "kinect/kinectManager.hpp"
#include "kinect/coordinateTransformer.hpp"

using namespace std::chrono_literals;

class KinectNode : public rclcpp::Node {
public:
  KinectNode() : Node("kinect_node")
  {
    // ===== Parameters =====
    color_resolution_ = this->declare_parameter<std::string>("color_resolution", "1080P");
    depth_mode_       = this->declare_parameter<std::string>("depth_mode", "NFOV_UNBINNED");
    color_format_     = this->declare_parameter<std::string>("color_format", "BGRA32");
    fps_              = this->declare_parameter<std::string>("fps", "30FPS");
    frame_id_color_   = this->declare_parameter<std::string>("frame_id_color", "camera_color_optical_frame");
    frame_id_depth_   = this->declare_parameter<std::string>("frame_id_depth", "camera_depth_optical_frame");

    RCLCPP_INFO(get_logger(), "Starting KinectNode (hardware driver)...");

    // ===== 1. Start Kinect Device =====
    kinect_ = std::make_unique<KinectManager>();
    kinect_->setConfig(color_resolution_, depth_mode_, color_format_, fps_);

    if (!kinect_->start()) {
      RCLCPP_FATAL(get_logger(), "Failed to start Azure Kinect device");
      throw std::runtime_error("k4a start failed");
    }

    // ===== 2. Initialize Transformer =====
    auto calibOpt = kinect_->getCalibration();
    if (calibOpt) {
      transformer_ = std::make_unique<ct::CoordinateTransformer>(*calibOpt);
      makeColorCameraInfo(*calibOpt, color_info_);
      makeDepthCameraInfo(*calibOpt, depth_info_);
      RCLCPP_INFO(get_logger(), "CoordinateTransformer initialized.");
    } else {
      RCLCPP_WARN(get_logger(), "No calibration available; publishing empty CameraInfo");
    }

    // ===== 3. Create Publishers =====
    pub_color_ = image_transport::create_publisher(this, "color/image_raw");
    pub_depth_ = image_transport::create_publisher(this, "depth/image_raw");
    pub_aligned_depth_ = image_transport::create_publisher(this, "depth/aligned_depth_to_color");

    // Camera Info & Raw Calibration (Latched QoS)
    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local();

    pub_color_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", qos_latched);
    pub_depth_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", qos_latched);
    pub_raw_calib_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("kinect/raw_calibration", qos_latched);

    // ===== 4. Publish Raw Calibration =====
    auto raw_data = kinect_->getRawCalibrationData();
    if (!raw_data.empty()) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = raw_data;
        pub_raw_calib_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published Raw Calibration Data (%zu bytes).", raw_data.size());
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to read Raw Calibration Data!");
    }

    // ===== 5. Timer =====
    const auto period = 33ms;
    timer_ = this->create_wall_timer(period, std::bind(&KinectNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "Kinect node started: %s %s %s %s",
      color_resolution_.c_str(), depth_mode_.c_str(), color_format_.c_str(), fps_.c_str());
  }

  ~KinectNode() override {
    if (kinect_) kinect_->stop();
  }

private:
  void onTimer()
  {
    auto result = kinect_->captureImages(1);
    if (!result.colorImage || !result.depthImage) return;

    // IMPORTANT: Use system clock for timestamp - both color and depth will share this timestamp
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    const rclcpp::Time stamp = clock.now();

    // Convert color image
    cv::Mat colorMat = imageProcessor::k4aImageToMat(result.colorImage, CV_8UC4);
    if (colorMat.empty()) {
      result.release();
      return;
    }

    // Convert depth image
    cv::Mat depthMat = imageProcessor::k4aImageToMat(result.depthImage, CV_16U);
    if (depthMat.empty()) {
      result.release();
      return;
    }

    // Publish Color
    auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", colorMat).toImageMsg();
    color_msg->header.stamp = stamp;
    color_msg->header.frame_id = frame_id_color_;
    pub_color_.publish(color_msg);
    if (color_info_.width > 0) {
      color_info_.header.stamp = stamp;
      color_info_.header.frame_id = frame_id_color_;
      pub_color_info_->publish(color_info_);
    }

    // Publish Raw Depth
    auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depthMat).toImageMsg();
    depth_msg->header.stamp = stamp;
    depth_msg->header.frame_id = frame_id_depth_;
    pub_depth_.publish(depth_msg);
    if (depth_info_.width > 0) {
      depth_info_.header.stamp = stamp;
      depth_info_.header.frame_id = frame_id_depth_;
      pub_depth_info_->publish(depth_info_);
    }

    // Generate and publish aligned depth
    if (transformer_) {
      ct::K4AImage alignedImg = transformer_->depthToColor(result.depthImage, result.colorImage);
      if (alignedImg) {
        cv::Mat alignedDepthMat = imageProcessor::k4aImageToMat(alignedImg.handle(), CV_16U);
        if (!alignedDepthMat.empty()) {
          auto aligned_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", alignedDepthMat).toImageMsg();
          aligned_msg->header.stamp = stamp;
          aligned_msg->header.frame_id = frame_id_color_;
          pub_aligned_depth_.publish(aligned_msg);
        }
      }
    }

    result.release();
  }

  // CameraInfo Helpers
  static void makeColorCameraInfo(const k4a_calibration_t& c, sensor_msgs::msg::CameraInfo& msg) {
    const auto& cc = c.color_camera_calibration;
    msg.width  = cc.resolution_width;
    msg.height = cc.resolution_height;
    const auto& p = cc.intrinsics.parameters.param;
    msg.k = { p.fx, 0.0, p.cx, 0.0, p.fy, p.cy, 0.0, 0.0, 1.0 };
    msg.p = { p.fx, 0.0, p.cx, 0.0, 0.0, p.fy, p.cy, 0.0, 0.0, 0.0, 1.0, 0.0 };
    msg.d = { p.k1, p.k2, p.p1, p.p2, p.k3, p.k4, p.k5, p.k6 };
    msg.distortion_model = "plumb_bob";
    msg.r = {1,0,0, 0,1,0, 0,0,1};
  }

  static void makeDepthCameraInfo(const k4a_calibration_t& c, sensor_msgs::msg::CameraInfo& msg) {
    const auto& dc = c.depth_camera_calibration;
    msg.width  = dc.resolution_width;
    msg.height = dc.resolution_height;
    const auto& p = dc.intrinsics.parameters.param;
    msg.k = { p.fx, 0.0, p.cx, 0.0, p.fy, p.cy, 0.0, 0.0, 1.0 };
    msg.p = { p.fx, 0.0, p.cx, 0.0, 0.0, p.fy, p.cy, 0.0, 0.0, 0.0, 1.0, 0.0 };
    msg.d = { p.k1, p.k2, p.p1, p.p2, p.k3, p.k4, p.k5, p.k6 };
    msg.distortion_model = "plumb_bob";
    msg.r = {1,0,0, 0,1,0, 0,0,1};
  }

private:
  // Parameters
  std::string color_resolution_, depth_mode_, color_format_, fps_;
  std::string frame_id_color_, frame_id_depth_;

  // Core components
  std::unique_ptr<KinectManager> kinect_;
  std::unique_ptr<ct::CoordinateTransformer> transformer_;

  // Publishers
  image_transport::Publisher pub_color_;
  image_transport::Publisher pub_depth_;
  image_transport::Publisher pub_aligned_depth_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_raw_calib_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::CameraInfo color_info_;
  sensor_msgs::msg::CameraInfo depth_info_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KinectNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
