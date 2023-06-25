#include "filter_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_particle_filter");
    FilterNode node;
    node.spin();
    return 0;
}