#pragma once

#include <map>

class CANManager {
public:
    CANManager(ros::NodeHandle* n, std::string& name) {
        CANPublisher = n->advertise<mrover::CAN>("can_requests", 1);
        assert(n->getParam("can/" + name + "/bus", bus));
        assert(bus.getType() == XmlRpc::XmlRpcValue::TypeInt);
        assert(n->getParam("can/" + name + "/id", id));
        assert(id.getType() == XmlRpc::XmlRpcValue::TypeInt);
    }

    void send_raw_data(uint64_t frame);

    int get_id() {
        return id;
    }

private:
    ros::Publisher CANPublisher;
    int bus;
    int id;
};