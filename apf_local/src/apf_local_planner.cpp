// apf_local_avoider.cpp

#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using octomap::point3d;

class APFLocalAvoider : public rclcpp::Node {
public:
    APFLocalAvoider() : Node("apf_local_avoider") {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&APFLocalAvoider::octomapCallback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/interpolated_path", 10,
            std::bind(&APFLocalAvoider::storePath, this, std::placeholders::_1));

        corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/corrected_path", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(100ms, std::bind(&APFLocalAvoider::timerCallback, this));
    }

private:
    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
    std::unordered_map<size_t, point3d> corrected_waypoints_;
    std::unordered_map<size_t, point3d> averaged_forces_;
    const double alpha = 0.5;  //controls



    nav_msgs::msg::Path original_path_;

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        auto tree = octomap_msgs::binaryMsgToMap(*msg);
        octree_.reset(dynamic_cast<octomap::OcTree *>(tree));
    }

    void storePath(const nav_msgs::msg::Path::SharedPtr msg) {
        original_path_ = *msg;
        corrected_waypoints_.clear();  // Clear previous corrections
        RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
    }
    

    void timerCallback() {
        if (!octree_ || original_path_.poses.empty()) return;
    
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("world", "simple_drone/base_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }
    
        point3d drone_pos(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);
    
        nav_msgs::msg::Path corrected_path;
        corrected_path.header.frame_id = "world";
        corrected_path.header.stamp = this->get_clock()->now();
    
        double distance_threshold = 3.0;
        // double max_shift_step = 0.4;
    
        for (size_t i = 0; i < original_path_.poses.size(); ++i) {
            const auto &original_pose = original_path_.poses[i];
            point3d current_wp = point3d(original_pose.pose.position.x,
                                         original_pose.pose.position.y,
                                         original_pose.pose.position.z);
    
            // computing distance to drone in world frame
            double distance = (current_wp - drone_pos).norm();
            point3d new_wp = current_wp;
    
            if (distance >= 1.0 && distance <= distance_threshold) {
                point3d rep_force = computeRepulsiveForce(current_wp, drone_pos);
            
                // Smooth the force using EMA
                if (averaged_forces_.count(i)) {
                    rep_force = rep_force * alpha + averaged_forces_[i] * (1 - alpha);

                }
            
                averaged_forces_[i] = rep_force;  // Update stored average
            
                // Limit the force
                // if (rep_force.norm() > max_shift_step)
                //     rep_force = rep_force.normalized() * max_shift_step;
            
                rep_force.z() = 1.3 * rep_force.z();  
                rep_force.x() = 1.5 * rep_force.x();  
                rep_force.y() = 1.5 * rep_force.y();
            
                new_wp += rep_force;
                corrected_waypoints_[i] = new_wp;
            }
            
            else if (corrected_waypoints_.count(i)) {
                // Keep previously modified waypoint
                new_wp = corrected_waypoints_[i];
            }
    
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = "world";
            ps.header.stamp = this->get_clock()->now();
            ps.pose.position.x = new_wp.x();
            ps.pose.position.y = new_wp.y();
            ps.pose.position.z = new_wp.z();
            ps.pose.orientation = original_pose.pose.orientation;
            corrected_path.poses.push_back(ps);
        }
    
        corrected_path_pub_->publish(corrected_path);
        RCLCPP_INFO(this->get_logger(), "Published corrected path with %zu poses", corrected_path.poses.size());
    }
    

    point3d computeRepulsiveForce(const point3d &wp, const point3d &drone_pos) {
        if (!octree_) return point3d(0, 0, 0);

        point3d force(0, 0, 0);
        double influence_radius = 3.0;
        // double k = 0.05;
        // double k = 0.06;
        // double k = 0.01; works well
        double k = 0.01;

        point3d min = drone_pos - point3d(influence_radius, influence_radius, influence_radius);
        point3d max = drone_pos + point3d(influence_radius, influence_radius, influence_radius);

        for (auto it = octree_->begin_leafs_bbx(min, max), end = octree_->end_leafs_bbx(); it != end; ++it) {
            if (!octree_->isNodeOccupied(*it)) continue;

            point3d obs(it.getX(), it.getY(), it.getZ());
            point3d dir = wp - obs;
            double dist = dir.norm();

            if (dist > influence_radius) continue;

            dir.normalize();  
            // double mag = k * (1.0 / dist - 1.0 / influence_radius) / (dist * dist);
            double mag = k * (1.0 / dist - 1.0 / influence_radius);
            force += dir * mag;
        }

        return force;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<APFLocalAvoider>());
    rclcpp::shutdown();
    return 0;
}
