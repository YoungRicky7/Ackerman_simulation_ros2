#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using SmoothPath = nav2_msgs::action::SmoothPath;

class SmootherClient:public rclcpp::Node{
public:
    SmootherClient():Node("smoother_client_node"){
      client_ = rclcpp_action::create_client<SmoothPath>(this, "smooth_path");
      subscription_ = this->create_subscription<nav_msgs::msg::Path>(
          "/unsmoothed_plan", 10, std::bind(&SmootherClient::unsmoothed_path_callback, this, _1));
    }
   rclcpp_action::Client<SmoothPath>::SharedPtr client_;
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;

    void unsmoothed_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        send_request(*msg);
    }

   void send_request(const nav_msgs::msg::Path &unsmoothed_path)
   {
      if(!client_->wait_for_action_server(10s))
      {
        RCLCPP_ERROR(this->get_logger(), "connect server failed!");
        return;
      }
      SmoothPath::Goal goal;
      goal.path = unsmoothed_path;
      goal.max_smoothing_duration.sec = 3;
      //goal.check_for_collisions = true;

      rclcpp_action::Client<SmoothPath>::SendGoalOptions options;
      options.goal_response_callback = std::bind(&SmootherClient::goal_response_callback,this,_1);
      options.feedback_callback = std::bind(&SmootherClient::feedback_callback,this,_1,_2);
      options.result_callback = std::bind(&SmootherClient::result_callback,this,_1);

      client_->async_send_goal(goal,options);   
   }

   void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<SmoothPath>> goal_handle)
   {
      if(goal_handle = nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "request is rejected!");
      }else
      {
        RCLCPP_INFO(this->get_logger(), "request is legal, about to handle request");
      }
   }

   void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<SmoothPath>> goal_handle, std::shared_ptr<const SmoothPath::Feedback>fd)
   {
   
   }

   void result_callback(const rclcpp_action::ClientGoalHandle<SmoothPath>::WrappedResult &result)
   {
      if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(this->get_logger(), "smoothed successfully!");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "response failed!");
      }
   }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
                                        
  auto node = std::make_shared<SmootherClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}