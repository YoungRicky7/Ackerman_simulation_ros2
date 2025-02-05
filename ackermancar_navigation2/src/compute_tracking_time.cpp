#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <thread>
#include <fstream>
#include <chrono>

using nav2_msgs::action::NavigateToPose;
using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;

enum class RecordType
{
    TRAJECTORY = 0,
    STEERING_ANGLE,
    SMOOTHED_PATH
};

geometry_msgs::msg::Quaternion transferTf2QuaternionToGeometryMsgQuaterion(const tf2::Quaternion &tf2_quaternion)
{
    geometry_msgs::msg::Quaternion geometry_quaternion;
    geometry_quaternion.x = tf2_quaternion.x();
    geometry_quaternion.y = tf2_quaternion.y();
    geometry_quaternion.z = tf2_quaternion.z();
    geometry_quaternion.w = tf2_quaternion.w();
    return geometry_quaternion;
}

class ControllerActionClient : public rclcpp::Node
{
public:
    ControllerActionClient(int callback_group_size) : Node("controller_action_client"),
                                                      callback_group_size_(callback_group_size)
    {
        initSubscritions();

        send_goal_options_.goal_response_callback =
            [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "goal has been sent to action server");
                start_time_ = std::chrono::steady_clock::now();
            }
        };

        send_goal_options_.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            (void)goal_handle;
            if (feedback)
            {
                RCLCPP_INFO(this->get_logger(), "remaining distance:%f", feedback->distance_remaining);
            }
        };

        send_goal_options_.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                end_time_ = std::chrono::steady_clock::now();

                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_);
                RCLCPP_INFO(this->get_logger(), "tracking time:%fs", duration.count() / 1000.0 / 1000.0);
                for (auto is_recording : is_recording_){
                    is_recording = false;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "fail to reach goal!");
            }
        };
    }

    bool connectToActionServer()
    {
        if (client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_INFO(this->get_logger(), "Connected to action server");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to action server");
            return false;
        }
    }

    void sendGoal(nav2_msgs::action::NavigateToPose::Goal goal)
    {
        client_->async_send_goal(goal, send_goal_options_);
    }

    void initRecordType(const std::vector<std::string> &record_types)
    {
        for (const auto &type : record_types)
        {
            if (type == "trajectory")
                is_recording_[static_cast<size_t>(RecordType::TRAJECTORY)] = true;
            else if (type == "steering_angle")
                is_recording_[static_cast<size_t>(RecordType::STEERING_ANGLE)] = true;
            else if (type == "smoothed_path")
                is_recording_[static_cast<size_t>(RecordType::SMOOTHED_PATH)] = true;
        }
    }

private:
    void steeringAngleCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        if (!is_recording_[static_cast<size_t>(RecordType::STEERING_ANGLE)])
            return;

        //RCLCPP_INFO(this->get_logger(), "Received steering angle: %f", msg->drive.steering_angle);
        steering_angle_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!is_recording_[static_cast<size_t>(RecordType::TRAJECTORY)])
            return;

        // RCLCPP_INFO(this->get_logger(), "Received odom: %f", msg->pose.pose.position.x);
        const char *home_dir = getenv("HOME");
        if (home_dir == nullptr){
              std::cerr << "Unable to get HOME environment variable" << std::endl;
            }
            std::string log_file_path = std::string(home_dir) + "/robot_odom_log.txt";
            std::ofstream log_file(log_file_path, std::ios::app);
            if (log_file.is_open())
            {
                if (log_file.tellp() == 0)
                {
                    log_file << "x y z qx qy qz qw\n";
                }

                log_file << msg->pose.pose.position.x << " "
                         << msg->pose.pose.position.y << " "
                         << msg->pose.pose.position.z << " "
                         << msg->pose.pose.orientation.x << " "
                         << msg->pose.pose.orientation.y << " "
                         << msg->pose.pose.orientation.z << " "
                         << msg->pose.pose.orientation.w << "\n";
                log_file.close();
            }
            else
            {
            RCLCPP_ERROR(this->get_logger(), "Unable to open log file");
        }
    }

    void smoothedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!is_recording_[static_cast<size_t>(RecordType::SMOOTHED_PATH)])
            return;

        RCLCPP_INFO(this->get_logger(), "Received smoothed path");
        const char *home_dir = getenv("HOME");
        if (home_dir == nullptr)
        {
            std::cerr << "Unable to get HOME environment variable" << std::endl;
        }
        std::string log_file_path = std::string(home_dir) + "/global_path.txt";
        std::ofstream log_file(log_file_path, std::ios::app);
        if (log_file.is_open())
        {
            if (log_file.tellp() == 0)
            {
                log_file << "x y z qx qy qz qw\n";
            }

            for (const auto &pose : msg->poses){
                log_file << pose.pose.position.x << " "
                        << pose.pose.position.y << " "
                        << pose.pose.position.z << " "
                        << pose.pose.orientation.x << " "
                        << pose.pose.orientation.y << " "
                        << pose.pose.orientation.z << " "
                        << pose.pose.orientation.w << "\n";                     
            }

            log_file.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open log file");
        }
    }

    void initSubscritions()
    {
        int index = 0;
        callback_groups_.resize(callback_group_size_);
        is_recording_.resize(callback_group_size_,false);
        sub_options_.resize(callback_group_size_);
        for (unsigned int i = 0; i < callback_group_size_; ++i)
        {
            callback_groups_[i] = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        }

        client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        for (unsigned int i = 0; i < callback_group_size_; ++i)
        {
            sub_options_[i].callback_group = callback_groups_[i];
            sub_options_[i].topic_stats_options.publish_period = std::chrono::milliseconds(1000);
        }

        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/neor_mini/cmd_ackermann",
            1,
            std::bind(&ControllerActionClient::steeringAngleCallback, this, std::placeholders::_1),
            sub_options_[index++]);

        smoothed_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan_smoothed",
            1,
            std::bind(&ControllerActionClient::smoothedPathCallback, this, std::placeholders::_1),
            sub_options_[index++]);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            1,
            std::bind(&ControllerActionClient::odomCallback, this, std::placeholders::_1),
            sub_options_[index++]);
    }

    std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
    std::vector<rclcpp::SubscriptionOptions> sub_options_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smoothed_path_sub_{nullptr};
    NavigateToPoseClient::SharedPtr client_{nullptr};
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr steering_angle_;
    std::vector<bool> is_recording_;
    unsigned int callback_group_size_{};
};

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "not enough arguments" << std::endl;
        return -1;
    }

    int n = 1;
    float goal_x = std::atof(argv[n++]);
    float goal_y = std::atof(argv[n++]);
    float goal_yaw = std::atof(argv[n++]);
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = goal_x;
    goal.pose.pose.position.y = goal_y;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    goal.pose.pose.orientation = transferTf2QuaternionToGeometryMsgQuaterion(q);

    std::vector<std::string> record_types;
    for (int i = 4; i < argc; ++i){
        record_types.push_back(argv[i]);
    }

    rclcpp::init(argc, argv);
    int callback_group_size = 10;
    auto node = std::make_shared<ControllerActionClient>(callback_group_size);
    
    node->initRecordType(record_types);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    if(!node->connectToActionServer()){
        RCLCPP_ERROR(node->get_logger(), "Failed to connect to action server");
        return -1;
    }
    node->sendGoal(goal);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}