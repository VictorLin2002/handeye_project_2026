#pragma once

/* ============================================================================
 * Coordinate Transformer (Azure Kinect DK + OpenCV) 
 * - Image alignment
 * - Point cloud generation
 * - [Restored] Point-to-Point transformation
 * ============================================================================ */

#include <k4a/k4a.h>
#include <optional>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <string>
#include <memory>
#include <cmath>                // for NAN
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>   // for cv::Point3f, cv::Point2f

namespace ct
{

/* ============================================================================
 * K4AImage - RAII wrapper for k4a_image_t
 *
 * Automatically manages Azure Kinect image lifetime.
 * Acquires resource on construction, releases on destruction.
 * ============================================================================ */
class K4AImage
{
public:
    K4AImage() = default;
    explicit K4AImage(k4a_image_t img) : m_image(img) {}
    
    ~K4AImage() { release(); }
    
    // Move semantics (transfer ownership)
    K4AImage(K4AImage &&other) noexcept : m_image(other.m_image) 
    { 
        other.m_image = nullptr; 
    }
    
    K4AImage &operator=(K4AImage &&other) noexcept
    {
        if (this != &other)
        {
            release();
            m_image = other.m_image;
            other.m_image = nullptr;
        }
        return *this;
    }
    
    // Disable copy (prevent double-free)
    K4AImage(const K4AImage &) = delete;
    K4AImage &operator=(const K4AImage &) = delete;
    
    // Accessors
    k4a_image_t handle() const noexcept { return m_image; }
    operator bool() const noexcept { return m_image != nullptr; }
    
    int width() const noexcept 
    { 
        return m_image ? k4a_image_get_width_pixels(m_image) : 0; 
    }
    
    int height() const noexcept 
    { 
        return m_image ? k4a_image_get_height_pixels(m_image) : 0; 
    }
    
    uint8_t* buffer() const noexcept 
    { 
        return m_image ? k4a_image_get_buffer(m_image) : nullptr; 
    }
    
    int stride() const noexcept 
    { 
        return m_image ? k4a_image_get_stride_bytes(m_image) : 0; 
    }
    
    k4a_image_t release() noexcept
    {
        k4a_image_t tmp = m_image;
        if (m_image)
        {
            k4a_image_release(m_image);
            m_image = nullptr;
        }
        return tmp;
    }

private:
    k4a_image_t m_image = nullptr;
};

/* ============================================================================
 * K4ATransformation - RAII wrapper for k4a_transformation_t
 *
 * Automatically manages transformation handle lifetime.
 * ============================================================================ */
class K4ATransformation
{
public:
    explicit K4ATransformation(const k4a_calibration_t &calib);
    ~K4ATransformation();
    
    K4ATransformation(const K4ATransformation &) = delete;
    K4ATransformation &operator=(const K4ATransformation &) = delete;
    K4ATransformation(K4ATransformation &&) = delete;
    K4ATransformation &operator=(K4ATransformation &&) = delete;

    k4a_transformation_t handle() const noexcept { return m_tf; }
    const k4a_calibration_t &calib() const noexcept { return m_calib; }

private:
    k4a_calibration_t m_calib{};
    k4a_transformation_t m_tf = nullptr;
};

/* ============================================================================
 * Point cloud structures
 * ============================================================================ */
struct PointXYZ
{
    float x, y, z;
    
    PointXYZ() : x(0), y(0), z(0) {}
    PointXYZ(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct PointXYZRGB
{
    float x, y, z;
    uint8_t r, g, b;
    
    PointXYZRGB() : x(0), y(0), z(0), r(0), g(0), b(0) {}
    PointXYZRGB(float x_, float y_, float z_, uint8_t r_, uint8_t g_, uint8_t b_)
        : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {}
};

/* ============================================================================
 * I/O utilities
 * ============================================================================ */
bool savePlyXYZ(const std::string &path, const std::vector<PointXYZ> &points);
bool savePlyXYZRGB(const std::string &path, const std::vector<PointXYZRGB> &points);

/* ============================================================================
 * ImageAligner : depth<->color alignment with RAII
 * ============================================================================ */
class ImageAligner
{
public:
    explicit ImageAligner(const K4ATransformation &tf) : m_tf(tf) {}

    K4AImage depthToColor(k4a_image_t depthImage, k4a_image_t colorImage) const;
    K4AImage colorToDepth(k4a_image_t colorImage, k4a_image_t depthImage) const;

private:
    const K4ATransformation &m_tf;
};

/* ============================================================================
 * PointCloudGenerator - Template-based unified generation
 * ============================================================================ */
class PointCloudGenerator
{
public:
    explicit PointCloudGenerator(const K4ATransformation &tf);

    // Unified interface
    std::vector<PointXYZRGB> generatePointCloudRGB(
        k4a_image_t depthImage,
        k4a_image_t colorImage,
        uint16_t zMinMm = 0,
        uint16_t zMaxMm = 0,
        bool useMedian5x5 = false) const;

    std::vector<PointXYZ> generatePointCloud(
        k4a_image_t depthImage,
        k4a_image_t colorImage,
        uint16_t zMinMm = 0,
        uint16_t zMaxMm = 0,
        bool useMedian5x5 = false) const;

private:
    const K4ATransformation &m_tf;
    ImageAligner m_aligner;
    
    template<typename PointT>
    std::vector<PointT> generatePointCloudImpl(
        k4a_image_t depthImage,
        k4a_image_t colorImage,
        uint16_t zMinMm,
        uint16_t zMaxMm,
        bool useMedian5x5,
        bool withColor) const;
};

/* ============================================================================
 * CoordinateTransformer : Facade with proper resource management
 * ============================================================================ */
class CoordinateTransformer
{
public:
    explicit CoordinateTransformer(const k4a_calibration_t &calib);

    /* Image alignment - returns RAII-wrapped images */
    K4AImage depthToColor(k4a_image_t depthImage, k4a_image_t colorImage) const
    {
        return m_aligner.depthToColor(depthImage, colorImage);
    }
    
    K4AImage colorToDepth(k4a_image_t colorImage, k4a_image_t depthImage) const
    {
        return m_aligner.colorToDepth(colorImage, depthImage);
    }

    /* Point cloud generation */
    std::vector<PointXYZRGB> generatePointCloudRGB(
        k4a_image_t depthImage,
        k4a_image_t colorImage,
        uint16_t zMinMm = 0,
        uint16_t zMaxMm = 0,
        bool useMedian5x5 = false) const
    {
        return m_generator.generatePointCloudRGB(depthImage, colorImage, zMinMm, zMaxMm, useMedian5x5);
    }

    std::vector<PointXYZ> generatePointCloud(
        k4a_image_t depthImage,
        k4a_image_t colorImage,
        uint16_t zMinMm = 0,
        uint16_t zMaxMm = 0,
        bool useMedian5x5 = false) const
    {
        return m_generator.generatePointCloud(depthImage, colorImage, zMinMm, zMaxMm, useMedian5x5);
    }

    /* File I/O - PLY format storage */
    bool savePlyXYZ(const std::string &path, const std::vector<PointXYZ> &points) const
    {
        return ct::savePlyXYZ(path, points);
    }

    bool savePlyXYZRGB(const std::string &path, const std::vector<PointXYZRGB> &points) const
    {
        return ct::savePlyXYZRGB(path, points);
    }

    /* ============================================================================
     * [Restored] Point-to-Point transformation - Interface as per your preference
     * ============================================================================ */

    // Input: 2D pixel coordinates (x, y) & depth (mm) -> Output: 3D coordinates (x, y, z) in mm
    // Defaults to Color Camera coordinate system (suitable for passing AprilTag pixel coordinates directly)
    cv::Point3f ICS2CCS(float x, float y, uint16_t depth_mm) const;

    // Input: 3D coordinates (x, y, z) in mm -> Output: 2D pixel coordinates (x, y)
    cv::Point2f CCS2ICS(float X, float Y, float Z) const;

    // Input: Color image pixel -> Output: Depth image pixel (used for parallax correction/lookup)
    // Requires the current raw depth image
    cv::Point2f pointColorToDepth(const cv::Point2f& point, k4a_image_t depth_img) const;


    // Get underlying K4A object
    const K4ATransformation &tf() const noexcept { return m_tf; }

private:
    K4ATransformation m_tf;
    ImageAligner m_aligner;
    PointCloudGenerator m_generator;
};

} /* namespace ct */