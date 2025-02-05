#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <thread>
#include <chrono>
#include <atomic>
struct Pose
{
    float x;
    float y;
    float yaw;
    Pose() {};
    Pose(float x, float y, float yaw) : x(x), y(y), yaw(yaw) {};
};

class SetGoalPose : public rclcpp::Node
{
public:
    SetGoalPose() : Node("set_goal_pose")
    {
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
    }

    void sendGoalPose()
    {
        std::atomic<int> n(0);
        while (n++<times_&&rclcpp::ok())
        {
            goal_publisher_->publish(start_pose_stamped_);
            goal_publisher_->publish(goal_pose_stamped_);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::cout<<"---send start and goal pose---"<<std::endl;
        }    
        std::cout<<"---test compeleted---"<<std::endl; 
    }

    void readPlanningInfo(const Pose& start_pose, const Pose& goal_pose)
    {
        start_pose_stamped_.header.frame_id = "map";
        start_pose_stamped_.pose.position.x = start_pose.x;
        start_pose_stamped_.pose.position.y = start_pose.y;
        start_pose_stamped_.pose.position.z = 0.0;
        tf2::Quaternion start_qtn;
        start_qtn.setRPY(0.0, 0.0, start_pose.yaw);
        start_pose_stamped_.pose.orientation = transferTf2QuaternionToGeometryMsgQuaterion(start_qtn);

        goal_pose_stamped_.header.frame_id = "map";
        goal_pose_stamped_.pose.position.x = goal_pose.x;
        goal_pose_stamped_.pose.position.y = goal_pose.y;
        goal_pose_stamped_.pose.position.z = 0.0;
        tf2::Quaternion goal_qtn;
        goal_qtn.setRPY(0.0, 0.0, goal_pose.yaw);
        goal_pose_stamped_.pose.orientation = transferTf2QuaternionToGeometryMsgQuaterion(goal_qtn);
    }

    geometry_msgs::msg::Quaternion transferTf2QuaternionToGeometryMsgQuaterion(const tf2::Quaternion& tf2_quaternion)
    {
        geometry_msgs::msg::Quaternion geometry_quaternion;
        geometry_quaternion.x = tf2_quaternion.x();
        geometry_quaternion.y = tf2_quaternion.y();
        geometry_quaternion.z = tf2_quaternion.z();
        geometry_quaternion.w = tf2_quaternion.w();
        return geometry_quaternion;
    }

    void setPlannintTimes(int times)
    {
        times_ = times;
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    geometry_msgs::msg::PoseStamped start_pose_stamped_;
    geometry_msgs::msg::PoseStamped goal_pose_stamped_;
    int times_;
};

int main(int argc, char **argv)
{
    if(argc!=8)
    {
        std::cerr<<"not enough arguments"<<std::endl;
        return -1;
    }

    int n = 1;
    float start_x = std::atof(argv[n++]);
    float start_y = std::atof(argv[n++]);
    float start_yaw = std::atof(argv[n++]);
    float goal_x = std::atof(argv[n++]);
    float goal_y = std::atof(argv[n++]);
    float goal_yaw = std::atof(argv[n++]);
    int times = std::atoi(argv[n++]);
    Pose start_pose(start_x, start_y, start_yaw);
    Pose goal_pose(goal_x, goal_y, goal_yaw);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetGoalPose>();
    node->readPlanningInfo(start_pose, goal_pose);
    node->setPlannintTimes(times);
    node->sendGoalPose();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
