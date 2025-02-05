#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>
#include <sstream>

enum class AlgorithmType : char {
    MPPI = 'M',
    DWA = 'D',
    TEB = 'T',
    RPP = 'R',
    VPC = 'V'
};

class PathVisualizer : public rclcpp::Node
{
public:
    PathVisualizer(const std::vector<std::string> &algorithm_names) : Node("path_visualizer")
    {
        for(auto algorithm_name : algorithm_names){
            algorithm_paths_hash_map_[algorithm_name] = nav_msgs::msg::Path();
            algorithm_paths_pubs_.push_back(this->create_publisher<nav_msgs::msg::Path>("/"+algorithm_name+"_trajectory", 10));
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathVisualizer::publishActualPath, this));
    }

private:
    void publishActualPath()
    {   
        static bool read_once = false;
        if(!read_once){
            const char *home_dir = getenv("HOME");
            std::string log_file_path = std::string(home_dir);
            for (auto algorithm : algorithm_paths_hash_map_)
            {
                log_file_path += "/" + algorithm.first + "_trajectory.txt";
                    
                std::ifstream log_file(log_file_path);
                if (!log_file.is_open()){
                    RCLCPP_ERROR(this->get_logger(), "Unable to open log file for reading");
                    return;
                }

                nav_msgs::msg::Path path_msg;
                path_msg.header.frame_id = "map";
                path_msg.header.stamp = this->get_clock()->now();

                std::string line;
                std::getline(log_file, line);

                while (std::getline(log_file, line))
                {
                    std::istringstream iss(line);
                    geometry_msgs::msg::PoseStamped pose;
                    iss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z >> pose.pose.orientation.x >> pose.pose.orientation.y >> pose.pose.orientation.z >> pose.pose.orientation.w;
                    path_msg.poses.push_back(pose);
                }
                algorithm_paths_hash_map_[algorithm.first] = path_msg;
            }
            read_once = true;
        }

        int index = 0;
        for (auto algorithm : algorithm_paths_hash_map_){
            algorithm_paths_pubs_[index++]->publish(algorithm.second);
        }
    }
        
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, nav_msgs::msg::Path> algorithm_paths_hash_map_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> algorithm_paths_pubs_;
};

int main(int argc, char **argv)
{
    if(argc < 2){
        std::cerr << "at least one algorithm name is needed" << std::endl;
        return -1;
    }

    std::vector<std::string> algorithm_names;
    for(int i = 1; i < argc; i++){
       
        switch(std::atoi(argv[i])){
            case static_cast<int>(AlgorithmType::MPPI):
                algorithm_names.push_back("mppi");
                break;
            case static_cast<int>(AlgorithmType::DWA):
                algorithm_names.push_back("dwa");
                break;
            case static_cast<int>(AlgorithmType::TEB):
                algorithm_names.push_back("teb");
                break;
            case static_cast<int>(AlgorithmType::RPP):
                algorithm_names.push_back("rpp");
                break;
            case static_cast<int>(AlgorithmType::VPC):
                algorithm_names.push_back("vpc");
                break;
            case 'A':
                algorithm_names.clear();
                algorithm_names.push_back("mppi");
                algorithm_names.push_back("dwa");
                algorithm_names.push_back("teb");
                algorithm_names.push_back("rpp");
                algorithm_names.push_back("vpc");
                break;
            default:
                std::cerr << "invalid algorithm name" << std::endl;
                return -1;
        }
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathVisualizer>(algorithm_names);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
