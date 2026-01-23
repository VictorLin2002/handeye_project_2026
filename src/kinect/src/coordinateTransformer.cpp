#include "kinect/coordinateTransformer.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <cmath> // Ensure NAN is available

namespace ct
{

/* ============================================================================
 * PLY I/O utilities - Binary format for fast I/O
 * ============================================================================ */
bool savePlyXYZ(const std::string &path, const std::vector<PointXYZ> &points)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open())
        return false;

    /* Write header */
    std::string header = "ply\n";
    header += "format binary_little_endian 1.0\n";
    header += "element vertex " + std::to_string(points.size()) + "\n";
    header += "property float x\n";
    header += "property float y\n";
    header += "property float z\n";
    header += "end_header\n";
    
    ofs.write(header.c_str(), header.size());

    /* Write binary data */
    for (const auto &p : points)
    {
        ofs.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
    }

    return true;
}

bool savePlyXYZRGB(const std::string &path, const std::vector<PointXYZRGB> &points)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open())
        return false;

    /* Write header */
    std::string header = "ply\n";
    header += "format binary_little_endian 1.0\n";
    header += "element vertex " + std::to_string(points.size()) + "\n";
    header += "property float x\n";
    header += "property float y\n";
    header += "property float z\n";
    header += "property uchar red\n";
    header += "property uchar green\n";
    header += "property uchar blue\n";
    header += "end_header\n";
    
    ofs.write(header.c_str(), header.size());

    /* Write binary data */
    for (const auto &p : points)
    {
        ofs.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&p.r), sizeof(uint8_t));
        ofs.write(reinterpret_cast<const char*>(&p.g), sizeof(uint8_t));
        ofs.write(reinterpret_cast<const char*>(&p.b), sizeof(uint8_t));
    }

    return true;
}

/* ============================================================================
 * K4ATransformation
 * ============================================================================ */
K4ATransformation::K4ATransformation(const k4a_calibration_t &calib)
    : m_calib(calib)
{
    m_tf = k4a_transformation_create(&m_calib);
    if (!m_tf)
        throw std::runtime_error("k4a_transformation_create failed");
}

K4ATransformation::~K4ATransformation()
{
    if (m_tf)
    {
        k4a_transformation_destroy(m_tf);
        m_tf = nullptr;
    }
}

/* ============================================================================
 * ImageAligner
 * ============================================================================ */
K4AImage ImageAligner::depthToColor(k4a_image_t depthImage, k4a_image_t colorImage) const
{
    if (!depthImage || !colorImage)
        return K4AImage();

    const int w = k4a_image_get_width_pixels(colorImage);
    const int h = k4a_image_get_height_pixels(colorImage);

    k4a_image_t aligned = nullptr;
    if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, w, h, w * (int)sizeof(uint16_t), &aligned) != K4A_RESULT_SUCCEEDED)
        return K4AImage();

    if (k4a_transformation_depth_image_to_color_camera(m_tf.handle(), depthImage, aligned) != K4A_RESULT_SUCCEEDED)
    {
        k4a_image_release(aligned);
        return K4AImage();
    }
    
    return K4AImage(aligned);
}

K4AImage ImageAligner::colorToDepth(k4a_image_t colorImage, k4a_image_t depthImage) const
{
    if (!depthImage || !colorImage)
        return K4AImage();

    const int w = k4a_image_get_width_pixels(depthImage);
    const int h = k4a_image_get_height_pixels(depthImage);

    k4a_image_t transformed = nullptr;
    if (k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, w, h, w * 4, &transformed) != K4A_RESULT_SUCCEEDED)
        return K4AImage();

    if (k4a_transformation_color_image_to_depth_camera(m_tf.handle(), depthImage, colorImage, transformed) != K4A_RESULT_SUCCEEDED)
    {
        k4a_image_release(transformed);
        return K4AImage();
    }
    
    return K4AImage(transformed);
}

/* ============================================================================
 * Helper functions
 * ============================================================================ */
static inline void copyMatToK4aImage(const cv::Mat &src16u, k4a_image_t dstDepth16)
{
    const int w = k4a_image_get_width_pixels(dstDepth16);
    const int h = k4a_image_get_height_pixels(dstDepth16);
    const int stride = k4a_image_get_stride_bytes(dstDepth16);
    uint8_t *dst = k4a_image_get_buffer(dstDepth16);

    for (int y = 0; y < h; ++y)
    {
        const uint8_t *srcRow = src16u.ptr<uint8_t>(y);
        std::memcpy(dst + y * stride, srcRow, w * sizeof(uint16_t));
    }
}

/* ============================================================================
 * PointCloudGenerator
 * ============================================================================ */
PointCloudGenerator::PointCloudGenerator(const K4ATransformation &tf)
    : m_tf(tf), m_aligner(tf) {}

template<typename PointT>
std::vector<PointT> PointCloudGenerator::generatePointCloudImpl(
    k4a_image_t depthImage,
    k4a_image_t colorImage,
    uint16_t zMinMm,
    uint16_t zMaxMm,
    bool useMedian5x5,
    bool withColor) const
{
    std::vector<PointT> out;
    if (!depthImage || !colorImage)
        return out;

    K4AImage depth_on_color = m_aligner.depthToColor(depthImage, colorImage);
    if (!depth_on_color)
        return out;

    const int w = depth_on_color.width();
    const int h = depth_on_color.height();

    cv::Mat depthAligned(h, w, CV_16U, depth_on_color.buffer(), depth_on_color.stride());

    cv::Mat depthFiltered;
    if (useMedian5x5)
        cv::medianBlur(depthAligned, depthFiltered, 5);
    else
        depthFiltered = depthAligned;

    K4AImage depthForPC;
    if (useMedian5x5)
    {
        k4a_image_t tmp = nullptr;
        if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, w, h, w * (int)sizeof(uint16_t), &tmp) != K4A_RESULT_SUCCEEDED)
            return out;
        
        depthForPC = K4AImage(tmp);
        copyMatToK4aImage(depthFiltered, depthForPC.handle());
    }
    else
    {
        depthForPC = std::move(depth_on_color);
    }

    k4a_image_t xyz_tmp = nullptr;
    if (k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, w, h, w * 3 * (int)sizeof(int16_t), &xyz_tmp) != K4A_RESULT_SUCCEEDED)
        return out;
    
    K4AImage xyz_image(xyz_tmp);

    if (k4a_transformation_depth_image_to_point_cloud(m_tf.handle(), depthForPC.handle(),
                                                      K4A_CALIBRATION_TYPE_COLOR, xyz_image.handle()) != K4A_RESULT_SUCCEEDED)
        return out;

    int16_t *pcbuf = reinterpret_cast<int16_t*>(xyz_image.buffer());
    out.reserve(static_cast<size_t>(w) * static_cast<size_t>(h) / 2);

    const float scale = 1.0f / 1000.0f;

    cv::Mat colorBGRA;
    if (withColor)
    {
        uint8_t *cbuf = k4a_image_get_buffer(colorImage);
        const int cstride = k4a_image_get_stride_bytes(colorImage);
        colorBGRA = cv::Mat(h, w, CV_8UC4, cbuf, cstride);
    }

    for (int v = 0; v < h; ++v)
    {
        const cv::Vec4b *rowC = withColor ? colorBGRA.ptr<cv::Vec4b>(v) : nullptr;
        const int base = v * w * 3;

        for (int u = 0; u < w; ++u)
        {
            const int idx3 = base + u * 3;
            const int16_t Xmm = pcbuf[idx3 + 0];
            const int16_t Ymm = pcbuf[idx3 + 1];
            const int16_t Zmm = pcbuf[idx3 + 2];

            if (Zmm <= 0 || 
                (zMinMm && Zmm < (int)zMinMm) || 
                (zMaxMm && Zmm > (int)zMaxMm))
                continue;

            const float x = Xmm * scale;
            const float y = Ymm * scale;
            const float z = Zmm * scale;

            if constexpr (std::is_same_v<PointT, PointXYZRGB>)
            {
                if (withColor && rowC)
                {
                    const cv::Vec4b &bgra = rowC[u];
                    out.emplace_back(x, y, z, bgra[2], bgra[1], bgra[0]);
                }
                else
                {
                    out.emplace_back(x, y, z, 0, 0, 0);
                }
            }
            else
            {
                out.emplace_back(x, y, z);
            }
        }
    }
    return out;
}

std::vector<PointXYZRGB> PointCloudGenerator::generatePointCloudRGB(
    k4a_image_t depthImage, k4a_image_t colorImage, uint16_t zMinMm, uint16_t zMaxMm, bool useMedian5x5) const
{
    return generatePointCloudImpl<PointXYZRGB>(depthImage, colorImage, zMinMm, zMaxMm, useMedian5x5, true);
}

std::vector<PointXYZ> PointCloudGenerator::generatePointCloud(
    k4a_image_t depthImage, k4a_image_t colorImage, uint16_t zMinMm, uint16_t zMaxMm, bool useMedian5x5) const
{
    return generatePointCloudImpl<PointXYZ>(depthImage, colorImage, zMinMm, zMaxMm, useMedian5x5, false);
}

/* ============================================================================
 * CoordinateTransformer
 * ============================================================================ */
CoordinateTransformer::CoordinateTransformer(const k4a_calibration_t &calib)
    : m_tf(calib), m_aligner(m_tf), m_generator(m_tf) {}


/* ============================================================================
 * [Restored] Point-to-Point Implementation
 * ============================================================================ */

cv::Point3f CoordinateTransformer::ICS2CCS(float x, float y, uint16_t depth_mm) const
{
    // 0 indicates invalid depth
    if (depth_mm == 0) return cv::Point3f(NAN, NAN, NAN);

    k4a_float2_t pixel = {x, y};
    k4a_float3_t point3d;
    int valid = 0;

    // Uses Color Camera parameters as the application scenario involves finding points on the Color image
    // This is most intuitive when working with Aligned Depth
    k4a_calibration_2d_to_3d(
        &m_tf.calib(),
        &pixel,
        static_cast<float>(depth_mm),
        K4A_CALIBRATION_TYPE_COLOR, 
        K4A_CALIBRATION_TYPE_COLOR, 
        &point3d,
        &valid
    );

    if (valid)
        return cv::Point3f(point3d.xyz.x, point3d.xyz.y, point3d.xyz.z); // unit: mm
    else
        return cv::Point3f(NAN, NAN, NAN);
}

cv::Point2f CoordinateTransformer::CCS2ICS(float X, float Y, float Z) const
{
    k4a_float3_t point3d = { X, Y, Z };
    k4a_float2_t pixel;
    int valid = 0;

    k4a_calibration_3d_to_2d(
        &m_tf.calib(),
        &point3d,
        K4A_CALIBRATION_TYPE_COLOR,
        K4A_CALIBRATION_TYPE_COLOR,
        &pixel,
        &valid
    );

    if (valid)
        return cv::Point2f(pixel.xy.x, pixel.xy.y);
    else
        return cv::Point2f(NAN, NAN);
}

cv::Point3f CoordinateTransformer::ICS2CCS_DepthToColor(float x_depth, float y_depth, uint16_t depth_mm) const
{
    // 0 indicates invalid depth
    if (depth_mm == 0) return cv::Point3f(NAN, NAN, NAN);

    k4a_float2_t pixel = {x_depth, y_depth};
    k4a_float3_t point3d;
    int valid = 0;

    // Source: Depth camera image coordinates
    // Target: Color camera 3D coordinates
    k4a_calibration_2d_to_3d(
        &m_tf.calib(),
        &pixel,
        static_cast<float>(depth_mm),
        K4A_CALIBRATION_TYPE_DEPTH,   // source: depth camera
        K4A_CALIBRATION_TYPE_COLOR,   // target: color camera frame
        &point3d,
        &valid
    );

    if (valid)
        return cv::Point3f(point3d.xyz.x, point3d.xyz.y, point3d.xyz.z); // unit: mm
    else
        return cv::Point3f(NAN, NAN, NAN);
}

cv::Point2f CoordinateTransformer::colorPixelToDepthPixel(float u_color, float v_color, float estimated_depth_mm) const
{
    // Step 1: Unproject color pixel to 3D point in color camera frame
    k4a_float2_t color_pixel = {u_color, v_color};
    k4a_float3_t point3d_color;
    int valid = 0;

    k4a_calibration_2d_to_3d(
        &m_tf.calib(),
        &color_pixel,
        estimated_depth_mm,
        K4A_CALIBRATION_TYPE_COLOR,
        K4A_CALIBRATION_TYPE_COLOR,
        &point3d_color,
        &valid
    );

    if (!valid) return cv::Point2f(NAN, NAN);

    // Step 2: Project 3D point to depth camera image
    k4a_float2_t depth_pixel;
    valid = 0;

    k4a_calibration_3d_to_2d(
        &m_tf.calib(),
        &point3d_color,
        K4A_CALIBRATION_TYPE_COLOR,   // source: color camera frame
        K4A_CALIBRATION_TYPE_DEPTH,   // target: depth camera image
        &depth_pixel,
        &valid
    );

    if (valid)
        return cv::Point2f(depth_pixel.xy.x, depth_pixel.xy.y);
    else
        return cv::Point2f(NAN, NAN);
}

cv::Point2f CoordinateTransformer::pointColorToDepth(const cv::Point2f& point, k4a_image_t depth_img) const
{
    if (depth_img == nullptr) {
        return cv::Point2f(NAN, NAN);
    }

    k4a_float2_t pixel = { point.x, point.y };
    k4a_float2_t transformed_pixel;
    int valid = 0;

    k4a_calibration_color_2d_to_depth_2d(
        &m_tf.calib(),
        &pixel,
        depth_img,
        &transformed_pixel,
        &valid
    );

    if (valid)
        return cv::Point2f(transformed_pixel.xy.x, transformed_pixel.xy.y);
    else
        return cv::Point2f(NAN, NAN);
}

} /* namespace ct */