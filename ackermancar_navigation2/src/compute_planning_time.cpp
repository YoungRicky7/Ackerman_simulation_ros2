#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <thread>
#include <chrono>

using nav2_msgs::action::NavigateToPose;
using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;

class PlannerClient : public rclcpp::Node
{
public:
    PlannerClient() : Node("planner_client")
    {

    }

    void workingThread()
    {
        
    }

private:
    std::chrono::steady_clock::time_point start_time_, end_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerClient>();
    std::thread thread(std::bind(&PlannerClient::workingThread, node));
    thread.detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
