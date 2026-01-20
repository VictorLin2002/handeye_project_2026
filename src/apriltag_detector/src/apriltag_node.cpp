#include <memory>
#include <functional>
#include <array>
#include <unordered_map>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"

#include "apriltag_detector/apriltagDetector.hpp"

class AprilTagNode : public rclcpp::Node
{
public:
  AprilTagNode() : rclcpp::Node("apriltag_node")
  {
    // Detection precision parameters
    // quad_decimate = 1.0: Full resolution (no downsampling) for maximum accuracy
    // quad_sigma = 0.8: Light Gaussian blur to reduce sensor noise while preserving edges
    // refine_edges = true: Enable subpixel edge refinement
    const double quad_decimate = this->declare_parameter("quad_decimate", 1.0);
    const double quad_sigma    = this->declare_parameter("quad_sigma", 0.8);
    const int    nthreads      = this->declare_parameter("nthreads", 5);
    const bool   refine_edges  = this->declare_parameter("refine_edges", true);

    detector_ = std::make_unique<AprilTagDetector>(quad_decimate, quad_sigma, nthreads, refine_edges);

    // Subscribe to color image (image_transport)
    sub_ = image_transport::create_subscription(
      this,
      "color/image_raw",
      std::bind(&AprilTagNode::onImage, this, std::placeholders::_1),
      "raw",
      rmw_qos_profile_sensor_data);

    // Debug image publisher
    pub_marked_ = image_transport::create_publisher(this, "apriltag/image_marked");

    // Publish ordered 2D points (sec,nsec prefix)
    pub_points_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "apriltag/ordered_points", 10);

    // Publish all detections (sec,nsec prefix)
    pub_all_tags_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "apriltag/all_detections", 10);

    RCLCPP_INFO(this->get_logger(), "AprilTag Node Ready. Subscribing to color/image_raw");
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv_bridge::CvImageConstPtr cvp;
    try
    {
      cvp = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat &img = cvp->image;
    const auto dets = detector_->detect(img);

    // Publish marked image for debugging
    if (pub_marked_.getNumSubscribers() > 0)
    {
      cv::Mat marked = detector_->drawDetections(img, dets, true, true, 2);
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", marked).toImageMsg();
      pub_marked_.publish(out_msg);
    }

    // IMPORTANT:
    // Use image header stamp ONLY. Do not mix with node clock time.
    const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);

    publishOrderedPoints(dets, stamp, pub_points_);
    publishAllDetections(dets, stamp, pub_all_tags_);
  }

  static void appendStampSecNsec(std::vector<double> &data, const rclcpp::Time &stamp)
  {
    const int64_t ns = stamp.nanoseconds();
    const int64_t sec = ns / 1000000000LL;
    const int64_t nsec = ns % 1000000000LL;
    data.push_back(static_cast<double>(sec));
    data.push_back(static_cast<double>(nsec));
  }

  void publishOrderedPoints(
    const std::vector<TagDetection> &dets,
    const rclcpp::Time &stamp,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub)
  {
    // Tag order: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9] - Dodecahedron with 10 tags
    static constexpr std::array<int, 10> kOrder{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static constexpr int kPointsPerTag = 5;  // center + 4 corners

    std::unordered_map<int, const TagDetection *> lookup;
    lookup.reserve(dets.size());
    for (const auto &det : dets)
    {
      lookup[det.id] = &det;
    }

    std_msgs::msg::Float64MultiArray out;

    // Format:
    // [sec, nsec, (tag1 center+corners uv ...), (tag3 ...), (tag0 ...), (tag2 ...)]
    // Each tag contributes 5 points (center + 4 corners), each point is (u,v) => 10 doubles.
    const size_t total = 2 + kOrder.size() * static_cast<size_t>(kPointsPerTag) * 2;

    out.data.reserve(total);
    appendStampSecNsec(out.data, stamp);

    auto appendPoint = [&](const cv::Point2f *pt) {
      if (pt)
      {
        out.data.push_back(static_cast<double>(pt->x));
        out.data.push_back(static_cast<double>(pt->y));
      }
      else
      {
        out.data.push_back(std::numeric_limits<double>::quiet_NaN());
        out.data.push_back(std::numeric_limits<double>::quiet_NaN());
      }
    };

    for (int tagId : kOrder)
    {
      auto it = lookup.find(tagId);
      if (it != lookup.end())
      {
        appendPoint(&it->second->center);
        for (const auto &corner : it->second->corners)
        {
          appendPoint(&corner);
        }
      }
      else
      {
        for (int i = 0; i < kPointsPerTag; ++i)
        {
          appendPoint(nullptr);
        }
      }
    }

    pub->publish(out);
  }

  void publishAllDetections(
    const std::vector<TagDetection> &dets,
    const rclcpp::Time &stamp,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub)
  {
    // Format:
    // [sec, nsec, num_tags,
    //  tag_id, center_x, center_y, c0x, c0y, c1x, c1y, c2x, c2y, c3x, c3y,
    //  ...]
    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(3 + dets.size() * 11);

    appendStampSecNsec(out.data, stamp);
    out.data.push_back(static_cast<double>(dets.size()));

    for (const auto &det : dets)
    {
      out.data.push_back(static_cast<double>(det.id));
      out.data.push_back(static_cast<double>(det.center.x));
      out.data.push_back(static_cast<double>(det.center.y));
      for (const auto &corner : det.corners)
      {
        out.data.push_back(static_cast<double>(corner.x));
        out.data.push_back(static_cast<double>(corner.y));
      }
    }

    pub->publish(out);
  }

private:
  std::unique_ptr<AprilTagDetector> detector_;

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_marked_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_points_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_all_tags_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagNode>());
  rclcpp::shutdown();
  return 0;
}
