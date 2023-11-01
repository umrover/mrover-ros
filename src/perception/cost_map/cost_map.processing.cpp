#include "cost_map.hpp"

#include "../point.hpp"
#include <algorithm>
#include <pstl/glue_execution_defs.h>

namespace mrover {

    /**
     * Calculate the costs of each point in pointcloud 
     * and stitch to occupancy grid.
     *
     * @param msg   Point cloud message
     */
    void CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg){
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        // Question: Determine number of points in pointcloud in msg
        int size = 0; //TODO
        std::for_each(std::execution::par_unseq, pointPtr, pointPtr + size, [&cutoff](Point& point)
        {
            float curv = point.curvature;
            int cost = 0;
            if (curv >= cutoff)
            {
                cost = 100;
            }
            // Question: How do we get the transformation matrix to get to the correct basis?
            
            

        });
        
    }   
}

/*
TODO:
- Nodelet takes in pointcloud
PARALLELIZE
- Get cost of point
- Transform point to nav basis
- Stitch to nav grid
END

*/