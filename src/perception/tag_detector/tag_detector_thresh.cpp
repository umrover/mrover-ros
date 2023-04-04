#include "tag_detector.hpp"

#include <cmath>
#include <vector>
#include <execution>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

namespace mrover {
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
    void runAdaptiveThresh(cv::Mat &grey, const cv::Ptr<cv::aruco::DetectorParameters> &params) {

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
                // cv::Mat thresh;
                threshold(grey, grey, currScale, params->adaptiveThreshConstant);
            }
        });
    }

    /**
     * Detect tags from raw image using OpenCV and calculate their screen space centers.
     * Tag pose information relative to the camera in 3D space is filled in when we receive point cloud data.
     *
     * @param msg
     */
    void TagDetectorNodelet::publishThresholdedImage() {
        /// 1. CONVERT TO GRAY
        convertToGrey(mImg, mGrayscale);

        /// 2. DETECT FIRST SET OF CANDIDATES
        runAdaptiveThresh(mGrayscale, mDetectorParams);

        mThreshMsg.header.seq = mSeqNum;
        mThreshMsg.header.stamp = ros::Time::now();
        mThreshMsg.header.frame_id = "zed2i_left_camera_frame";
        mThreshMsg.height = mGrayscale.rows;
        mThreshMsg.width = mGrayscale.cols;
        mThreshMsg.encoding = sensor_msgs::image_encodings::MONO8;
        mThreshMsg.step = mGrayscale.step;
        mThreshMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        size_t size = mThreshMsg.step * mThreshMsg.height;
        mThreshMsg.data.resize(size);
        std::copy(std::execution::par_unseq, mGrayscale.data, mGrayscale.data + size, mThreshMsg.data.begin());
        mThreshPub.publish(mThreshMsg);
    }
}