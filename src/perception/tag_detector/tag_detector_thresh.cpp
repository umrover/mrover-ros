#include "tag_detector.hpp"

#include "filter.hpp"
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


constexpr size_t IMAGE_WIDTH_WARN_SIZE = 640;
constexpr size_t IMAGE_HEIGHT_WARN_SIZE = 480;

/**
  *
  */
cv::aruco::DetectorParameters::DetectorParameters()
    : adaptiveThreshWinSizeMin(10),
      adaptiveThreshWinSizeMax(23),
      adaptiveThreshWinSizeStep(10),
      adaptiveThreshConstant(50),
      minMarkerPerimeterRate(0.03),
      maxMarkerPerimeterRate(4.),
      polygonalApproxAccuracyRate(0.03),
      minCornerDistanceRate(0.05),
      minDistanceToBorder(3),
      minMarkerDistanceRate(0.05),
      cornerRefinementMethod(CORNER_REFINE_NONE),
      cornerRefinementWinSize(5),
      cornerRefinementMaxIterations(30),
      cornerRefinementMinAccuracy(0.1),
      markerBorderBits(1),
      perspectiveRemovePixelPerCell(4),
      perspectiveRemoveIgnoredMarginPerCell(0.13),
      maxErroneousBitsInBorderRate(0.35),
      minOtsuStdDev(5.0),
      errorCorrectionRate(0.6),
      aprilTagQuadDecimate(0.0),
      aprilTagQuadSigma(0.0),
      aprilTagMinClusterPixels(5),
      aprilTagMaxNmaxima(10),
      aprilTagCriticalRad( (float)(10* CV_PI /180) ),
      aprilTagMaxLineFitMse(10.0),
      aprilTagMinWhiteBlackDiff(5),
      aprilTagDeglitch(0),
      detectInvertedMarker(false){}


/**
  * @brief Create a new set of DetectorParameters with default values.
  */
cv::Ptr<cv::aruco::DetectorParameters> cv::aruco::DetectorParameters::create() {
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
    return params;
}

/**
  * @brief Threshold input image using adaptive thresholding
  */
static void threshold(cv::InputArray _in, cv::OutputArray _out, int winSize, double constant) {

    CV_Assert(winSize >= 3);
    if(winSize % 2 == 0) winSize++; // win size must be odd
    cv::adaptiveThreshold(_in, _out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, winSize, constant);
}

/**
  * @brief Convert input image to gray if it is a 3-channels image
  */
void convertToGrey(cv::InputArray _in, cv::OutputArray _out) {

    CV_Assert(_in.type() == CV_8UC1 || _in.type() == CV_8UC3);

    if(_in.type() == CV_8UC3)
        cvtColor(_in, _out, cv::COLOR_BGR2GRAY);
    else
        _in.copyTo(_out);
}

/**
 * @brief Initial steps on finding square candidates
 */
void detectInitialCandidates(cv::Mat &grey, const cv::Ptr<cv::aruco::DetectorParameters> &params) {

    CV_Assert(params->adaptiveThreshWinSizeMin >= 3 && params->adaptiveThreshWinSizeMax >= 3);
    CV_Assert(params->adaptiveThreshWinSizeMax >= params->adaptiveThreshWinSizeMin);
    CV_Assert(params->adaptiveThreshWinSizeStep > 0);

    // number of window sizes (scales) to apply adaptive thresholding
    int nScales =  (params->adaptiveThreshWinSizeMax - params->adaptiveThreshWinSizeMin) /
                      params->adaptiveThreshWinSizeStep + 1;

    ////for each value in the interval of thresholding window sizes
    parallel_for_(cv::Range(0, nScales), [&](const cv::Range& range) {
        const int begin = range.start;
        const int end = range.end;

        for (int i = begin; i < end; i++) {
            int currScale = params->adaptiveThreshWinSizeMin + i * params->adaptiveThreshWinSizeStep;
            // threshold
            cv::Mat thresh;
            threshold(grey, thresh, currScale, params->adaptiveThreshConstant);
            cv::imshow("test", thresh);
            cv::waitKey(1);
        }
    });
}

/**
 * @brief Detect square candidates in the input image
 */
void detectCandidates(cv::InputArray _image, const cv::Ptr<cv::aruco::DetectorParameters> &_params) {

    cv::Mat image = _image.getMat();
    CV_Assert(image.total() != 0);

    /// 1. CONVERT TO GRAY
    cv::Mat grey;
    convertToGrey(image, grey);

    /// 2. DETECT FIRST SET OF CANDIDATES
    detectInitialCandidates(grey, _params);
}

/**
 * Detect tags from raw image using OpenCV and calculate their screen space centers.
 * Tag pose information relative to the camera in 3D space is filled in when we receive point cloud data.
 *
 * @param msg
 */
void TagDetectorNode::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
    if (!mEnableDetections) return;

    bool isInSim = false;
    mNh.getParam("use_sim_time", isInSim);

    if (!isInSim && (msg->width <= IMAGE_WIDTH_WARN_SIZE || msg->height <= IMAGE_HEIGHT_WARN_SIZE)) {
        ROS_WARN("Input image is below 640x480 resolution. Tag detection may be poor");
    }

    ROS_DEBUG("Got image %d", msg->header.seq);

    try {
        cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Detect the tag vertices in screen space and their respective ids
        // {mCorners, mIds} are the outputs from OpenCV

        // change image to greyscale
        CV_Assert(!imagePtr->image.empty());
        cv::Mat grey;

        convertToGrey(imagePtr->image, grey);

        detectCandidates(grey, mDetectorParams);
        //display thresh test
        cv::aruco::detectMarkers(imagePtr->image, mDictionary, mCorners, mIds, mDetectorParams);



        if (!mTags.empty()) {
            cv::aruco::drawDetectedMarkers(imagePtr->image, mCorners, mIds);
        }

        if (mPublishImages) {
            mImgPub.publish(imagePtr->toImageMsg());
        }

        mSeqNum++;
    } catch (cv_bridge::Exception const& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (cv::Exception const& e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}