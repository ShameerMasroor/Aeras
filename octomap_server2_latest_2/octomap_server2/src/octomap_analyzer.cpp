#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <chrono>
#include <filesystem>
namespace fs = std::filesystem;



class OctomapAnalyzer : public rclcpp::Node {
public:
    OctomapAnalyzer() : Node("octomap_analyzer") {
        // Store the simulation start time
        // start_time_ = this->now();

        // Subscribe to the full Octomap topic
        subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", rclcpp::QoS(100),
            std::bind(&OctomapAnalyzer::octomapCallback, this, std::placeholders::_1));        
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /octomap_binary");

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm *time_info = std::localtime(&now_time);

        // Create the directory if it doesn't exist
        fs::path folder = "Exploration Runs";
        if (!fs::exists(folder)) {
            fs::create_directories(folder);
        }

        // Create the timestamped filename inside the folder
        char filename[100];
        std::strftime(filename, sizeof(filename), "exploration_log_%Y%m%d_%H-%M.csv", time_info);


        fs::path filepath = folder / filename;

        // Open the CSV file with the timestamped name
        csv_file_.open(filepath, std::ios::app);


// Open the CSV file with the timestamped name
       

        if (!csv_file_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file!");
        } else {
            csv_file_ << "ElapsedTime(s),ExplorationPercentage\n";
        }
    }

    ~OctomapAnalyzer() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
    std::ofstream csv_file_; // File stream for logging
    // rclcpp::Time start_time_; // Store simulation start time

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        // Convert Octomap message to OcTree
        std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
        if (!tree_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message!");
            return;
        }

        auto tree = dynamic_cast<octomap::OcTree*>(tree_ptr.get());
        if (!tree) {
            RCLCPP_ERROR(this->get_logger(), "Converted tree is not an OcTree!");
            return;
        }

        // Compute elapsed time in seconds since simulation start
        // rclcpp::Time current_time = this->now();
        rclcpp::Time current_time=msg->header.stamp;
        double elapsed_time = (current_time).seconds();

        int total_cells = 0;

        // Define the exploration area
        double area_min_x = -25.0;
        double area_max_x = 25.0;
        double area_min_y = -25.0;
        double area_max_y = 25.0;
        double area_max_z = 3.0;
        double area_min_z = 0.0;
        double expected_volume = 7500.0; // 50.0 * 50.0 * 4.0


        // double desired_resolution = 0.9;
        // unsigned int max_depth = tree->getTreeDepth();
        // unsigned int d_target = max_depth - std::log2(desired_resolution / tree->getResolution());
        
        // size_t total_cells = 0;
        
        // for (octomap::OcTree::leaf_iterator n = tree->begin_leafs(d_target);
        //      n != tree->end_leafs(); ++n) {
        

        //     // double x_cur = n.getX();
        //     // double y_cur = n.getY();
        //     // double z_cur = n.getZ();
        
        //     // if (x_cur >= area_min_x && x_cur <= area_max_x &&
        //     //     y_cur >= area_min_y && y_cur <= area_max_y &&
        //     //     z_cur >= area_min_z && z_cur <= area_max_z) {
        //     //     total_cells++;
        //     // }
        // }
        // Fix percentage calculation

        double volume = 0;

        for(octomap::OcTree::leaf_iterator n = tree->begin_leafs(tree->getTreeDepth()); n != tree->end_leafs(); ++n) {
            double x_cur = n.getX();
            double y_cur = n.getY();
            double z_cur = n.getZ();
    

            double half_size = n.getSize() / 2.0;
            if ((x_cur - half_size >= area_min_x) && (x_cur + half_size <= area_max_x) &&
                (y_cur - half_size >= area_min_y) && (y_cur + half_size <= area_max_y) &&
                (z_cur - half_size >= area_min_z) && (z_cur + half_size <= area_max_z)) {
                
                double voxel_size = n.getSize();
                volume += std::pow(voxel_size, 3);
            }

            
            // volume += pow(n.getSize(), 3);
        }
        // RCLCPP_INFO(this->get_logger(), "Size: %f",
        //             voxels);
        // RCLCPP_INFO(this->get_logger(), "Old Volume: %f",
        // old_volume);

        // int voxels_in_tree = static_cast<int>(tree->getTreeDepth());
        double percentage = (volume / expected_volume) * 100.0;
        

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm *time_info = std::localtime(&now_time);
        char time_str[20];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", time_info);

        RCLCPP_INFO(this->get_logger(), "Timestamp: %s | Percentage: %f",
                    time_str, percentage);

        // Write to CSV with correct column order
        if (csv_file_.is_open()) {
            csv_file_ << std::fixed << std::setprecision(2) << elapsed_time << ","
                      << std::setprecision(2) << percentage << "\n";
            csv_file_.flush(); 
        } else {
            RCLCPP_ERROR(this->get_logger(), "CSV file is not open!");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapAnalyzer>());
    rclcpp::shutdown();
    return 0;
}
