#pragma once

/* ============================================================================
 *  - Manages apriltag_detector_t / apriltag_family_t lifecycle
 * ============================================================================ */

#include <apriltag.h>
#include <tag36h11.h>
#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <string>

struct TagDetection
{
	int id{ -1 };                              /* tag id */
	cv::Point2f center{ 0.f, 0.f };            /* tag center in pixels */
	std::array<cv::Point2f, 4> corners{};      /* p0..p3 in pixels (same order as apriltag) */
	int hamming{ 0 };                           /* error correction bits */
	double decisionMargin{ 0.0 };               /* quality metric */
};

class AprilTagDetector
{
public:
	/* ---------------------------------------------------------------------
	 * Construction / Destruction (RAII boundary = detector + family)
	 * --------------------------------------------------------------------- */
	AprilTagDetector(
		double quadDecimate = 0.5,
		double quadSigma = 1.4,
		int nthreads = 2,
		bool refineEdges = true);
	~AprilTagDetector();

	AprilTagDetector(const AprilTagDetector &) = delete;
	AprilTagDetector &operator=(const AprilTagDetector &) = delete;
	AprilTagDetector(AprilTagDetector &&) = delete;
	AprilTagDetector &operator=(AprilTagDetector &&) = delete;

	/* ---------------------------------------------------------------------
	 * Parameter update
	 * --------------------------------------------------------------------- */
	void setQuadDecimate(double v) noexcept;
	void setQuadSigma(double v) noexcept;
	void setThreads(int n) noexcept;
	void setRefineEdges(bool v) noexcept;

	/* ---------------------------------------------------------------------
	 * Detection (gray or color input is fine; will convert to gray internally)
	 * --------------------------------------------------------------------- */
	std::vector<TagDetection> detect(const cv::Mat &inputImage);

	/* ---------------------------------------------------------------------
	 * Drawing utilities
	 *  - drawDetections: returns a BGR canvas with edges/ids; input can be gray or color
	 *  - showDetections: convenience wrapper that does imshow + waitKey
	 * --------------------------------------------------------------------- */
	cv::Mat drawDetections(const cv::Mat &inputImage, const std::vector<TagDetection> &dets,
		bool drawIds = true, bool drawEdges = true, int thickness = 2);

	void showDetections(const std::string &windowName, const cv::Mat &bgrImage, int delayMs = 1);

private:
	apriltag_detector_t *m_td{ nullptr };
	apriltag_family_t *m_tf{ nullptr };

	/* Internal helpers */
	static cv::Mat ensureGray(const cv::Mat &img);
	static void ensureBgrCanvas(const cv::Mat &srcGray, cv::Mat &dstBgr);
};
