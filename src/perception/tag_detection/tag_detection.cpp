#include "tag_detection.hpp"

#include <opencv2/aruco.hpp>

std::vector<uint8_t> g_ids;
std::vector<std::vector<cv::Point2f>> g_corners;

void detectTags(PointCloudPtr const& pointCloud, Detections& detections) {
    // Non-owning
    cv::Mat image{static_cast<int>(pointCloud->height), static_cast<int>(pointCloud->width),
                  CV_8UC3, &pointCloud->front(), sizeof(PointCloud::PointType)};
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto parameters = cv::aruco::DetectorParameters::create();

    cv::aruco::detectMarkers(image, dictionary, g_ids, g_corners, parameters);

    detections.clear();
    for (size_t i = 0; i < g_ids.size(); ++i) {
        cv::Point2f centerFloat = std::accumulate(g_corners[i].begin(), g_corners[i].end(), cv::Point2f{}) / 4;
        cv::Point2i center{static_cast<int>(std::lround(centerFloat.x)), static_cast<int>(std::lround(centerFloat.y))};

        PointCloud::PointType const& point = pointCloud->at(center.x, center.y);
        SE pose{point.x, point.y, point.z, 0, 0, 0};
        detections.emplace(g_ids[i], Tag{center, position});
        //        detections.emplace(g_ids[i],
        //                           Tag{g_corners[i], cv::Point3f{0, 0, 0}})
    }
}