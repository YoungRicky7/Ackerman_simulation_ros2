#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <functional>
#include <thread>
class AMCLAutoInit:public rclcpp::Node{
public:
    AMCLAutoInit():Node("amcl_auto_init"){
      robot_init_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom/ground_truth", 10, std::bind(&AMCLAutoInit::robot_init_odom_callback, this, std::placeholders::_1));
      initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_init_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  bool initial_pose_published_{false};
  const int sleep_time_before_init_{3};
  void robot_init_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!initial_pose_published_)
    {
      std::this_thread::sleep_for(std::chrono::seconds(sleep_time_before_init_));
      geometry_msgs::msg::PoseWithCovarianceStamped initialpose;
      initialpose.header = msg->header;
      initialpose.pose.pose = msg->pose.pose;
      initialpose.pose.covariance = msg->pose.covariance;
      initialpose_pub_->publish(initialpose);
      // std::cout << "intt pose " << initialpose.pose.pose.position.x << " " << initialpose.pose.pose.position.y;
      initial_pose_published_ = true;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
                                        
  rclcpp::spin(std::make_shared<AMCLAutoInit>());
                                        
  rclcpp::shutdown();
  return 0;
}