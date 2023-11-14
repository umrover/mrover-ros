#include "cost_map.hpp"
#include "../point.hpp"

#include <algorithm>
#include <pstl/glue_execution_defs.h>
#include <se3.hpp>

namespace mrover {

    /**
     * Stitches local occupancy grid to global occupancy grid
     *
     * @param msg   Local Occupancy Grid Mesasge
     */
    void CostMapNodelet::occupancyGridCallback(nav_msgs::OccupancyGrid const& msg) {

        // Make sure message is valid
        assert(msg.info.width > 0);
        assert(msg.info.height > 0);
        assert(msg.info.resolution > 0);

        // Using tf_tree, get the transformation

        SE3 base_to_map = SE3::fromTfTree(tf_buffer, "base_link", "map");
        SE3 zed_to_base = SE3::fromTfTree(tf_buffer, "zed2i_left_camera_frame", "base_link");
        Eigen::Matrix4d transformMatrix = zed_to_base.matrix() * base_to_map.matrix(); // This is the matrix transform from zed to the map

        auto* pointPtr = &msg.data;
        std::for_each(std::execution::par_unseq, &msg.data, &msg.data + msg.info.width, [&](int& point) {

        });
    }
} // namespace mrover
