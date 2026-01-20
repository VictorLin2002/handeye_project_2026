#include "kinect/kinectManager.hpp"
#include <cstring>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

/* ============================================================================
 * Static lookup maps (string -> SDK enum)
 * ============================================================================ */
const std::map<std::string, k4a_color_resolution_t> KinectManager::resolutionMap =
{
    {"720P",  K4A_COLOR_RESOLUTION_720P},
    {"1080P", K4A_COLOR_RESOLUTION_1080P},
    {"1440P", K4A_COLOR_RESOLUTION_1440P},
    {"1536P", K4A_COLOR_RESOLUTION_1536P},
    {"2160P", K4A_COLOR_RESOLUTION_2160P},
    {"3072P", K4A_COLOR_RESOLUTION_3072P}
};

const std::map<std::string, k4a_depth_mode_t> KinectManager::depthModeMap =
{
    {"NFOV_2X2BINNED", K4A_DEPTH_MODE_NFOV_2X2BINNED},
    {"NFOV_UNBINNED",  K4A_DEPTH_MODE_NFOV_UNBINNED},
    {"WFOV_2X2BINNED", K4A_DEPTH_MODE_WFOV_2X2BINNED},
    {"WFOV_UNBINNED",  K4A_DEPTH_MODE_WFOV_UNBINNED},
    {"PASSIVE_IR",     K4A_DEPTH_MODE_PASSIVE_IR}
};

const std::map<std::string, k4a_image_format_t> KinectManager::colorFormatMap =
{
    {"MJPG",   K4A_IMAGE_FORMAT_COLOR_MJPG},
    {"NV12",   K4A_IMAGE_FORMAT_COLOR_NV12},
    {"YUY2",   K4A_IMAGE_FORMAT_COLOR_YUY2},
    {"BGRA32", K4A_IMAGE_FORMAT_COLOR_BGRA32}
};

const std::map<std::string, k4a_fps_t> KinectManager::fpsMap =
{
    {"5FPS",  K4A_FRAMES_PER_SECOND_5},
    {"15FPS", K4A_FRAMES_PER_SECOND_15},
    {"30FPS", K4A_FRAMES_PER_SECOND_30}
};

/* ============================================================================
 * imageProcessor utility
 * ============================================================================ */
cv::Mat imageProcessor::k4aImageToMat(k4a_image_t image, int colorFormat)
{
    if (image == nullptr) return cv::Mat();

    size_t width = k4a_image_get_width_pixels(image);
    size_t height = k4a_image_get_height_pixels(image);
    if (width == 0 || height == 0) return cv::Mat();
    
    size_t stride = k4a_image_get_stride_bytes(image);
    uint8_t *buffer = k4a_image_get_buffer(image);

    cv::Mat mat(static_cast<int>(height), static_cast<int>(width), colorFormat, buffer, stride);
    return mat.clone(); 
}

/* ============================================================================
 * KinectManager
 * ============================================================================ */
KinectManager::KinectManager()
{
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true; 
}

KinectManager::~KinectManager()
{
    stop();
}

void KinectManager::setConfig(
    const std::string &colorResolutionStr,
    const std::string &depthModeStr,
    const std::string &colorFormatStr,
    const std::string &fpsStr)
{
    if (auto it = resolutionMap.find(colorResolutionStr); it != resolutionMap.end())
        config.color_resolution = it->second;
    else
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;

    if (auto it = depthModeMap.find(depthModeStr); it != depthModeMap.end())
        config.depth_mode = it->second;
    else
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    if (auto it = colorFormatMap.find(colorFormatStr); it != colorFormatMap.end())
        config.color_format = it->second;
    else
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

    if (auto it = fpsMap.find(fpsStr); it != fpsMap.end())
        config.camera_fps = it->second;
    else
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        
    config.synchronized_images_only = true;
}

const k4a_device_configuration_t &KinectManager::getConfig() const noexcept
{
    return config;
}

bool KinectManager::start()
{
    if (running) return true;

    if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "[KinectManager] Failed to open Azure Kinect device." << std::endl;
        device = nullptr;
        return false;
    }

    if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "[KinectManager] Failed to start camera." << std::endl;
        k4a_device_close(device);
        device = nullptr;
        return false;
    }

    running = true;
    return true;
}

void KinectManager::stop()
{
    if (!running && device == nullptr) return;

    if (device != nullptr)
    {
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
        device = nullptr;
    }
    running = false;
}

bool KinectManager::isRunning() const noexcept
{
    return running;
}

std::optional<k4a_capture_t> KinectManager::captureFrame(int timeoutMs)
{
    if (!running || device == nullptr) return std::nullopt;

    k4a_capture_t cap = nullptr;
    k4a_wait_result_t wr = k4a_device_get_capture(device, &cap, timeoutMs);
    
    if (wr == K4A_WAIT_RESULT_SUCCEEDED && cap != nullptr)
    {
        return cap; 
    }
    return std::nullopt;
}

void KinectManager::releaseCapture(k4a_capture_t capture) noexcept
{
    if (capture != nullptr)
    {
        k4a_capture_release(capture);
    }
}

std::optional<k4a_image_t> KinectManager::getColorImage(k4a_capture_t capture)
{
    if (capture == nullptr) return std::nullopt;

    k4a_image_t color = k4a_capture_get_color_image(capture);
    if (color == nullptr) return std::nullopt;

    const int width = k4a_image_get_width_pixels(color);
    const int height = k4a_image_get_height_pixels(color);
    const int stride = k4a_image_get_stride_bytes(color);
    const size_t size = k4a_image_get_size(color);
    uint8_t *src = k4a_image_get_buffer(color);

    k4a_image_t out = nullptr;
    if (k4a_image_create(k4a_image_get_format(color), width, height, stride, &out) == K4A_RESULT_SUCCEEDED)
    {
        std::memcpy(k4a_image_get_buffer(out), src, size);
        k4a_image_set_device_timestamp_usec(out, k4a_image_get_device_timestamp_usec(color));
        k4a_image_release(color); 
        return out;
    }
    
    k4a_image_release(color);
    return std::nullopt;
}

std::optional<k4a_image_t> KinectManager::getDepthImage(k4a_capture_t capture)
{
    if (capture == nullptr) return std::nullopt;

    k4a_image_t depth = k4a_capture_get_depth_image(capture);
    if (depth == nullptr) return std::nullopt;

    const int width = k4a_image_get_width_pixels(depth);
    const int height = k4a_image_get_height_pixels(depth);
    const int stride = k4a_image_get_stride_bytes(depth);
    const size_t size = k4a_image_get_size(depth);
    uint8_t *src = k4a_image_get_buffer(depth);

    k4a_image_t out = nullptr;
    if (k4a_image_create(k4a_image_get_format(depth), width, height, stride, &out) == K4A_RESULT_SUCCEEDED)
    {
        std::memcpy(k4a_image_get_buffer(out), src, size);
        k4a_image_set_device_timestamp_usec(out, k4a_image_get_device_timestamp_usec(depth));
        k4a_image_release(depth);
        return out;
    }
    
    k4a_image_release(depth);
    return std::nullopt;
}

std::optional<k4a_calibration_t> KinectManager::getCalibration() const
{
    if (device == nullptr) return std::nullopt;

    k4a_calibration_t calib{};
    if (k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calib) != K4A_RESULT_SUCCEEDED)
    {
        return std::nullopt;
    }
    return calib;
}

// [關鍵修正] 修改回傳類型為 k4a_buffer_result_t
std::vector<uint8_t> KinectManager::getRawCalibrationData() const
{
    if (device == nullptr) return {};

    size_t size = 0;
    
    // [FIX] 第一次呼叫：預期回傳 TOO_SMALL (因為 buffer 是 nullptr)
    k4a_buffer_result_t res = k4a_device_get_raw_calibration(device, nullptr, &size);
    
    // 如果回傳值不是 TOO_SMALL，或者大小為 0，表示失敗
    if (res != K4A_BUFFER_RESULT_TOO_SMALL || size == 0) {
        std::cerr << "[KinectManager] Failed to get raw calibration size." << std::endl;
        return {};
    }

    std::vector<uint8_t> raw_data(size);
    
    // 第二次呼叫：預期回傳 SUCCEEDED
    res = k4a_device_get_raw_calibration(device, raw_data.data(), &size);
    
    if (res != K4A_BUFFER_RESULT_SUCCEEDED) {
        std::cerr << "[KinectManager] Failed to get raw calibration data." << std::endl;
        return {};
    }
    
    return raw_data;
}

void CaptureResult::release()
{
    if (colorImage) { k4a_image_release(colorImage); colorImage = nullptr; }
    if (depthImage) { k4a_image_release(depthImage); depthImage = nullptr; }
    if (capture)    { KinectManager::releaseCapture(capture); capture = nullptr; }
}

CaptureResult KinectManager::captureImages(int /*maxAttempts unused*/)
{
    CaptureResult result;
    int32_t timeout_ms = 100; 

    auto capOpt = captureFrame(timeout_ms); 
    
    if (capOpt)
    {
        result.capture = *capOpt;
        auto colOpt = getColorImage(result.capture);
        auto depOpt = getDepthImage(result.capture);

        if (colOpt && depOpt)
        {
            result.colorImage = *colOpt;
            result.depthImage = *depOpt;
            return result;
        }
        
        if (colOpt) k4a_image_release(*colOpt);
        if (depOpt) k4a_image_release(*depOpt);
    }

    return result;
}