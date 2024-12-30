#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <thread>
#include <chrono>

using nav2_msgs::action::NavigateToPose;
using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;

class ControllerActionClient : public rclcpp::Node
{
public:
    ControllerActionClient() : Node("controller_action_client")
    {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        while (!client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
        }
    }

    void workingThread()
    {
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");

                start_time_ = std::chrono::steady_clock::now();
            }
        };

        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            (void)goal_handle; 
            if (feedback)
            {
                RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f", feedback->distance_remaining);
            }
        };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                end_time_ = std::chrono::steady_clock::now();

                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - start_time_);
                RCLCPP_INFO(this->get_logger(), "规划时间:%lldms", duration.count());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "目标执行失败");
            }
        };

        while (rclcpp::ok())
        {
            double goal_x, goal_y, goal_theta;
            std::cin >> goal_x >> goal_y >> goal_theta;

            nav2_msgs::action::NavigateToPose::Goal target_pose;
            target_pose.pose.pose.position.x = goal_x;
            target_pose.pose.pose.position.y = goal_y;
            target_pose.pose.pose.position.z = 0.0;
            target_pose.pose.header.frame_id = "map";

            tf2::Quaternion q;
            q.setRPY(0, 0, goal_theta);
            geometry_msgs::msg::Quaternion q_msg;
            q_msg.x = q.x();
            q_msg.y = q.y();
            q_msg.z = q.z();
            q_msg.w = q.w();

            target_pose.pose.pose.orientation = q_msg;

            client_->async_send_goal(target_pose, send_goal_options);
        }
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::chrono::steady_clock::time_point start_time_, end_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerActionClient>();
    std::thread thread(std::bind(&ControllerActionClient::workingThread, node));
    thread.detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
