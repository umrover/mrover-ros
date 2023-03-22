#include "tag_detector.hpp"

#include "filter.hpp"
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <vector>

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
        CV_Assert(!_imagePtr->image.empty());
        Mat grey;
        convertToGrey(_image.getMat(), grey);

        // detect marker candidates

        std::vector< std::vector< Point2f > > candidates;
        std::vector< std::vector< Point > > contours;
        std::vector< int > ids;

        std::vector< std::vector< std::vector< Point2f > > > candidatesSet;
        std::vector< std::vector< std::vector< Point > > > contoursSet;
        detectCandidates(grey, candidatesSet, countoursSet, mDetectorParams);
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

/**
  * @brief Convert input image to gray if it is a 3-channels image
  */
static void convertToGrey(InputArray _in, OutputArray _out) {

    CV_Assert(_in.type() == CV_8UC1 || _in.type() == CV_8UC3);

    if(_in.type() == CV_8UC3)
        cvtColor(_in, _out, COLOR_BGR2GRAY);
    else
        _in.copyTo(_out);
}

/**
 * @brief Detect square candidates in the input image
 */
static void detectCandidates(InputArray _image, vector< vector< vector< Point2f > > >& candidatesSetOut,
                              vector< vector< vector< Point > > >& contoursSetOut, const Ptr<DetectorParameters> &_params) {

    Mat image = _image.getMat();
    CV_Assert(image.total() != 0);

    /// 1. CONVERT TO GRAY
    Mat grey;
    convertToGrey(image, grey);

    std::vector< std::vector< Point2f > > candidates;
    std::vector< std::vector< Point > > contours;
    /// 2. DETECT FIRST SET OF CANDIDATES
    detectInitialCandidates(grey, candidates, contours, _params);
}

/**
 * @brief Initial steps on finding square candidates
 */
static void detectInitialCandidates(const Mat &grey, vector< vector< Point2f > > &candidates,
                                     vector< vector< Point > > &contours,
                                     const Ptr<DetectorParameters> &params) {

    CV_Assert(params->adaptiveThreshWinSizeMin >= 3 && params->adaptiveThreshWinSizeMax >= 3);
    CV_Assert(params->adaptiveThreshWinSizeMax >= params->adaptiveThreshWinSizeMin);
    CV_Assert(params->adaptiveThreshWinSizeStep > 0);

    // number of window sizes (scales) to apply adaptive thresholding
    int nScales =  (params->adaptiveThreshWinSizeMax - params->adaptiveThreshWinSizeMin) /
                      params->adaptiveThreshWinSizeStep + 1;

    std::vector< std::vector< std::vector< Point2f > > > candidatesArrays((size_t) nScales);
    std::vector< std::vector< std::vector< Point > > > contoursArrays((size_t) nScales);

    ////for each value in the interval of thresholding window sizes
    parallel_for_(Range(0, nScales), [&](const Range& range) {
        const int begin = range.start;
        const int end = range.end;

        for (int i = begin; i < end; i++) {
            int currScale = params->adaptiveThreshWinSizeMin + i * params->adaptiveThreshWinSizeStep;
            // threshold
            Mat thresh;
            _threshold(grey, thresh, currScale, params->adaptiveThreshConstant);
            cv::imshow("test", thresh);
            cv::waitKey(0);
        }
    });
}