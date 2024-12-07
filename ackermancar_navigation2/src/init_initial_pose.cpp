#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <algorithm>

class PoseReader : public rclcpp::Node
{
public:
    PoseReader(std::string robot_name)
        : Node("pose_reader"), robot_name_(robot_name)
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10, std::bind(&PoseReader::model_states_callback, this, std::placeholders::_1));
    }

private:
    void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {
        // 找到机器人模型的索引
        auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
        if (it != msg->name.end())
        {
            auto index = std::distance(msg->name.begin(), it);
            auto position = msg->pose[index].position;
            auto orientation = msg->pose[index].orientation;
            RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", orientation.x, orientation.y, orientation.z, orientation.w);
        }
    }
    std::string robot_name_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseReader>("hunter2"));
    rclcpp::shutdown();
    return 0;
}