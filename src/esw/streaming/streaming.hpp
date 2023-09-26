#pragma once

#include <opencv2/cudacodec.hpp>

struct Streamer {
    cv::Ptr<cv::cudacodec::VideoWriter> m_writer;

    Streamer();
};