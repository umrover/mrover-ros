#include <map>
#include <variant>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <geometry_msgs/Twist.h>

using namespace std::string_literals;

constexpr static int CONTROL_HZ = 50;

static ros::Duration const EXPIRATION_DURATION{1};

std::vector PUBLISHER_TOPICS{"joystick_cmd_vel"s, "controller_cmd_vel"s, "simulator_cmd_vel"s, "navigation_cmd_vel"s};

ros::Publisher finalTwistPub;

template<class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};

struct InputPublisher {
    ros::Time lastReceived;
    ros::Subscriber sub;
    std::variant<geometry_msgs::Twist> input;
};

std::map<std::string, InputPublisher> inputPublishers;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "superstructure");
    ros::NodeHandle nh;

    for (std::string const& topic: PUBLISHER_TOPICS) {
        auto& [lastReceived, sub, input] = inputPublishers[topic];
        sub = nh.subscribe<geometry_msgs::Twist>(topic, 1, [&](geometry_msgs::TwistConstPtr const& twist) {
            input = *twist;
            lastReceived = ros::Time::now();
        });
    }

    finalTwistPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Timer controlTimer = nh.createTimer(ros::Duration{1.0 / CONTROL_HZ}, [&](auto const&) {
        geometry_msgs::Twist finalTwist;
        for (auto const& [_, inputPublisher]: inputPublishers) {
            if (ros::Time::now() - inputPublisher.lastReceived > EXPIRATION_DURATION) continue;

            std::visit(overloaded{
                               [&](geometry_msgs::Twist const& twist) {
                                   finalTwist.linear.x += twist.linear.x;
                                   finalTwist.angular.z += twist.angular.z;
                               }},
                       inputPublisher.input);
        }
        finalTwistPub.publish(finalTwist);
    });

    ros::spin();

    return EXIT_SUCCESS;
}