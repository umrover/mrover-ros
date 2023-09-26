#include "streaming.hpp"

Streamer::Streamer() {
    m_writer = cv::cudacodec::createVideoWriter()
}
