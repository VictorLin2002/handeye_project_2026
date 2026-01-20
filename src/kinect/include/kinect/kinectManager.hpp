#pragma once

/* ============================================================================
 * KinectManager
 *  - Manages Azure Kinect device lifecycle and frame acquisition
 * ============================================================================ */

#include <k4a/k4a.h>
#include <map>
#include <string>
#include <optional>
#include <opencv2/core.hpp>

/* Optional small utility namespace for image conversions */
namespace imageProcessor
{
	/* Convert k4a_image_t to cv::Mat (cloned). colorFormat is CV_8UC4 for BGRA32, etc. */
	cv::Mat k4aImageToMat(k4a_image_t image, int colorFormat);
}

/* ============================================================================
 * CaptureResult - Encapsulates captured image data with resource management
 * ============================================================================ */
struct CaptureResult
{
	k4a_capture_t capture = nullptr;
	k4a_image_t colorImage = nullptr;
	k4a_image_t depthImage = nullptr;

	bool isValid() const
	{
		return capture && colorImage && depthImage;
	}

	void release();
};

class KinectManager
{
public:
	/* ---------------------------------------------------------------------
	 * Construction / Destruction (RAII boundary is the device handle)
	 * --------------------------------------------------------------------- */
	KinectManager();
	~KinectManager();

	KinectManager(const KinectManager &) = delete;
	KinectManager &operator=(const KinectManager &) = delete;
	KinectManager(KinectManager &&) = delete;
	KinectManager &operator=(KinectManager &&) = delete;

	/* ---------------------------------------------------------------------
	 * Configuration
	 * - setConfig accepts human-readable strings and maps them to SDK enums
	 * - call start() after setting config to open device and start cameras
	 * --------------------------------------------------------------------- */
	void setConfig(
		const std::string &colorResolutionStr,
		const std::string &depthModeStr,
		const std::string &colorFormatStr,
		const std::string &fpsStr);

	const k4a_device_configuration_t &getConfig() const noexcept;

	/* ---------------------------------------------------------------------
	 * Device lifecycle (explicit start/stop around RAII-held device_)
	 * --------------------------------------------------------------------- */
	bool start();
	void stop();
	bool isRunning() const noexcept;

	/* ---------------------------------------------------------------------
	 * Frame acquisition
	 *  - captureFrame returns an owned k4a_capture_t on success (caller releases)
	 *  - getColorImage / getDepthImage return cloned k4a_image_t (caller releases)
	 *  - captureImages bundles all three with retry logic
	 * --------------------------------------------------------------------- */
	std::optional<k4a_capture_t> captureFrame(int timeoutMs = 1000);
	static void releaseCapture(k4a_capture_t capture) noexcept;

	std::optional<k4a_image_t> getColorImage(k4a_capture_t capture);
	std::optional<k4a_image_t> getDepthImage(k4a_capture_t capture);

	/* Capture images with automatic retry logic
	 * Returns CaptureResult containing capture, color, and depth images.
	 * Use isValid() to check if capture succeeded.
	 * maxAttempts: number of retry attempts before giving up (default 15)
	 */
	CaptureResult captureImages(int maxAttempts = 15);

	/* ---------------------------------------------------------------------
	 * Calibration
	 * --------------------------------------------------------------------- */
	std::optional<k4a_calibration_t> getCalibration() const;
	std::vector<uint8_t> getRawCalibrationData() const;

public:
	/* Public static lookup maps (declared here, defined in .cpp) */
	static const std::map<std::string, k4a_color_resolution_t> resolutionMap;
	static const std::map<std::string, k4a_depth_mode_t> depthModeMap;
	static const std::map<std::string, k4a_image_format_t> colorFormatMap;
	static const std::map<std::string, k4a_fps_t> fpsMap;

private:
	/* Device and configuration */
	k4a_device_t device{ nullptr };
	k4a_device_configuration_t config{};
	bool running{ false };
};
