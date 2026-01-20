#include "apriltag_detector/apriltagDetector.hpp"
#include <sstream>
#include <cstring>

/* ============================================================================
 * Construction / Destruction
 * ============================================================================ */
AprilTagDetector::AprilTagDetector(double quadDecimate, double quadSigma, int nthreads, bool refineEdges)
{
	m_td = apriltag_detector_create();
	m_tf = tag36h11_create();

	m_td->quad_decimate = quadDecimate;
	m_td->quad_sigma = quadSigma;
	m_td->nthreads = nthreads;
	m_td->refine_edges = refineEdges ? 1 : 0;

	// Subpixel refinement parameters (critical for reducing 3D jitter)
	m_td->decode_sharpening = 0.25;  // Edge sharpening for better corner localization

	apriltag_detector_add_family(m_td, m_tf);
}

AprilTagDetector::~AprilTagDetector()
{
	if (m_td != nullptr)
	{
		apriltag_detector_destroy(m_td);
		m_td = nullptr;
	}
	if (m_tf != nullptr)
	{
		tag36h11_destroy(m_tf);
		m_tf = nullptr;
	}
}

/* ============================================================================
 * Parameter setters
 * ============================================================================ */
void AprilTagDetector::setQuadDecimate(double v) noexcept
{
	if (m_td) m_td->quad_decimate = v;
}

void AprilTagDetector::setQuadSigma(double v) noexcept
{
	if (m_td) m_td->quad_sigma = v;
}

void AprilTagDetector::setThreads(int n) noexcept
{
	if (m_td) m_td->nthreads = n;
}

void AprilTagDetector::setRefineEdges(bool v) noexcept
{
	if (m_td) m_td->refine_edges = v ? 1 : 0;
}

/* ============================================================================
 * Helpers
 * ============================================================================ */
cv::Mat AprilTagDetector::ensureGray(const cv::Mat &img)
{
	if (img.empty())
	{
		return cv::Mat();
	}
	if (img.type() == CV_8UC1)
	{
		return img;
	}
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	return gray;
}

void AprilTagDetector::ensureBgrCanvas(const cv::Mat &srcGray, cv::Mat &dstBgr)
{
	if (dstBgr.empty() || dstBgr.type() != CV_8UC3 ||
		dstBgr.cols != srcGray.cols || dstBgr.rows != srcGray.rows)
	{
		cv::cvtColor(srcGray, dstBgr, cv::COLOR_GRAY2BGR);
	}
}

/* ============================================================================
 * Detection
 * ============================================================================ */
std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat &inputImage)
{
	std::vector<TagDetection> out;

	cv::Mat gray = ensureGray(inputImage);
	if (gray.empty())
	{
		return out;
	}


    image_u8_t *imgU8 = image_u8_create(gray.cols, gray.rows);
    if (imgU8 == nullptr)
    {
        return out;
    }


    const int w = gray.cols;
    const int h = gray.rows;
    const size_t srcStride = static_cast<size_t>(gray.step);   // OpenCV 每列位元組數
    const size_t dstStride = static_cast<size_t>(imgU8->stride); // Apriltag stride，對 create() 來說等於 w
    const uint8_t *src = gray.ptr<uint8_t>(0);
    uint8_t *dst = imgU8->buf;

    for (int y = 0; y < h; ++y)
    {
        std::memcpy(dst + y * dstStride, src + y * srcStride, static_cast<size_t>(w));
    }

	zarray_t *detections = apriltag_detector_detect(m_td, imgU8);

	const int detCount = zarray_size(detections);
	out.reserve(detCount);
	for (int i = 0; i < detCount; ++i)
	{
		apriltag_detection_t *det = nullptr;
		zarray_get(detections, i, &det);

		TagDetection r;
		r.id = det->id;
		r.center = cv::Point2f(static_cast<float>(det->c[0]), static_cast<float>(det->c[1]));
		for (int j = 0; j < 4; ++j)
		{
			r.corners[j] = cv::Point2f(
				static_cast<float>(det->p[j][0]),
				static_cast<float>(det->p[j][1]));
		}
		r.hamming = det->hamming;
		r.decisionMargin = det->decision_margin;

		out.emplace_back(r);
	}

	apriltag_detections_destroy(detections);
	image_u8_destroy(imgU8);

	return out;
}

/* ============================================================================
 * Drawing utilities
 * ============================================================================ */
cv::Mat AprilTagDetector::drawDetections(const cv::Mat &inputImage, const std::vector<TagDetection> &dets,
	bool drawIds, bool drawEdges, int thickness)
{
	cv::Mat gray = ensureGray(inputImage);
	cv::Mat canvas;
	if (gray.empty())
	{
		return canvas; /* empty */
	}
	ensureBgrCanvas(gray, canvas);

	for (const auto &d : dets)
	{
		/* edges */
		if (drawEdges)
		{
			for (int j = 0; j < 4; ++j)
			{
				const cv::Point p1(static_cast<int>(d.corners[j].x), static_cast<int>(d.corners[j].y));
				const cv::Point p2(static_cast<int>(d.corners[(j + 1) % 4].x), static_cast<int>(d.corners[(j + 1) % 4].y));
				cv::line(canvas, p1, p2, cv::Scalar(255, 0, 0), thickness);
				// label this corner (0-3)
                std::stringstream ssCorner;
                ssCorner << j;
                const cv::Point textPt = p1 + cv::Point(5, -5);
                cv::putText(canvas, ssCorner.str(), textPt,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
			}
		}

		/* id at center */
		if (drawIds)
		{
			std::stringstream ss;
			ss << "ID:" << d.id;
			const cv::Point c(static_cast<int>(d.center.x), static_cast<int>(d.center.y));
			cv::putText(canvas, ss.str(), c, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
		}
	}

	return canvas;
}

void AprilTagDetector::showDetections(const std::string &windowName, const cv::Mat &bgrImage, int delayMs)
{
	cv::imshow(windowName, bgrImage);
	cv::waitKey(delayMs);
}
