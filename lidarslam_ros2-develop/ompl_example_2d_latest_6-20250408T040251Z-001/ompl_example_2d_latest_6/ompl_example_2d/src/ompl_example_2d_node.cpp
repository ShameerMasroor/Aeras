#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp> 
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "ompl_example_2d/ompl_example_2d.hpp"
#include "omp.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
using std::vector;
using std::pair;
using std::make_pair;

typedef octomap::point3d point3d;
// nav_msgs::msg::OccupancyGrid globalMap;
octomap_msgs::msg::Octomap globalMap_octo;
// octomap::OcTree globalMap;
std::shared_ptr<octomap::OcTree> globalMap_octree;
bool mapChanged(false);
bool mapChanged_octo(false);
// point3d startstate(0.0,0.0,0.0); //we can maybe change this to match the startstate of our drone.
int path_changed=0;
const double PI = 3.1415926;
double resolution = 0.9;
std::shared_ptr<octomap::OcTree> thermal_tree = std::make_shared<octomap::OcTree>(resolution);
 //correct this

struct sensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                InitialVector = point3d(1.0, 0.0, 0.0);
                InitialVector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                SensorRays.push_back(InitialVector);
        }
    }
};
sensorModel lidarsensor(360,90,2*PI,PI/2,20);
sensorModel thermalsensor(120,45,(2*PI)/3,PI/4,20);

// class MapSubscriber : public rclcpp::Node
// {
//   public:
//     MapSubscriber()
//     : Node("map_subscriber")
//     {
//       // subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//       // "map", 10, std::bind(&MapSubscriber::map_callback, this, _1));

//       subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
//       "/octomap_binary", 10, std::bind(&MapSubscriber::map_callback, this, _1));
//     }

//   private:
//     mutable std::shared_ptr<octomap::OcTree> octomap_;
//     void map_callback(const octomap_msgs::msg::Octomap & msg) const
//     {
//       octomap_ = std::make_shared<octomap::OcTree>(*dynamic_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(msg)));
//       globalMap = octomap_;
//       std::cout << "map subscribed\n";
//       mapChanged = true;
//     }
//     rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
// };

// class MapSubscriber : public rclcpp::Node
// {
// public:
//     MapSubscriber() : Node("map_subscriber")
//     {
//         subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
//             "/octomap_binary", 10, std::bind(&MapSubscriber::map_callback, this, _1));
//     }
//     // std::shared_ptr<octomap::OcTree> MapSubscriber::getOctoMap() {
//     //   std::lock_guard<std::mutex> lock(octomap_mutex_);
//     //   return octomap_;
//     // }

// private:
//     // std::shared_ptr<octomap::OcTree> octomap_;
//     // std::mutex map_mutex;

//     void map_callback(const octomap_msgs::msg::Octomap &msg)
//     {
//         // std::lock_guard<std::mutex> lock(map_mutex);
//         // auto map_ptr = dynamic_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(msg));
//         // if (!map_ptr)
//         // {
//         //     RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree.");
//         //     return;
//         // }
//         // octomap_ = std::make_shared<octomap::OcTree>(*map_ptr);
//         // globalMap = octomap_; // Ensure thread-safe global access
//         globalMap=msg;
//         RCLCPP_INFO(this->get_logger(), "Map subscribed successfully.");
//         mapChanged = true; // Ensure thread-safe global access
//     }

//     rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
// };



class PathPublisher : public rclcpp::Node
{
  public:
    PathPublisher()
    : Node("path_publisher")
    {
      subscription_octo = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", 10, std::bind(&PathPublisher::map_callback_octo, this, _1));
      // subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PathPublisher::map_callback, this, _1));
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    //   obstacle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle", 10);
      frontiers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);
      candidates_pub_= this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidates", 10);
      thermal_candidates_pub_= this->create_publisher<visualization_msgs::msg::MarkerArray>("/thermal_candidates", 10);
      thermal_frontiers_pub_= this->create_publisher<visualization_msgs::msg::MarkerArray>("/thermal_frontiers", 10);
      thermal_pub_= this->create_publisher<visualization_msgs::msg::MarkerArray>("/thermal_occupied", 10);
      path_request_= this->create_subscription<std_msgs::msg::Bool>("/find_path",10,std::bind(&PathPublisher::path_callback,this,_1));

      something_detected_sub_= this->create_subscription<std_msgs::msg::Bool>("/something_detected",10,std::bind(&PathPublisher::detection_callback,this,_1));

      heat_signature_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>("/heat_signature_angle",10,std::bind(&PathPublisher::heat_signature_callback,this,_1));

      yaw_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/yaw_debug", 10);

      // rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
      rclcpp::Clock::SharedPtr clock = this->get_clock();
      this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
      this->buffer_->setUsingDedicatedThread(true);
      this->m_tfListener = std::make_shared<tf2_ros::TransformListener>(
            *buffer_, this, false);

      // add a rectangular marker to visualize the obstacle 
      // obstacle_.header.frame_id = "map";
      // obstacle_.type = visualization_msgs::msg::Marker::CUBE;
      // obstacle_.pose.position.x = 5.05;
      // obstacle_.pose.position.y = -0.5;
      // obstacle_.pose.position.z = 0.0;
      // obstacle_.pose.orientation.x = 0.0;
      // obstacle_.pose.orientation.y = 0.0;
      // obstacle_.pose.orientation.z = 0.0;
      // obstacle_.pose.orientation.w = 1.0;
      // obstacle_.scale.x = 0.1;
      // obstacle_.scale.y = 9.0;
      // obstacle_.scale.z = 0.05;
      // obstacle_.color.r = 1.0;
      // obstacle_.color.g = 0.0;
      // obstacle_.color.b = 0.0;
      // obstacle_.color.a = 0.5;
    // Check this out later.
      timer_ = this->create_wall_timer(
      100ms, std::bind(&PathPublisher::timer_callback, this));

      heat_signature_angle = std::make_shared<std_msgs::msg::Float32>(); //initialization
      something_detected = std::make_shared<std_msgs::msg::Bool>(); //initialization
      something_detected->data = false;
      lidar_frontiers = std::make_shared<std::vector<std::vector<point3d>>>();
      thermal_frontiers = std::make_shared<std::vector<std::vector<point3d>>>();
    }


  private:
    // octomap_msgs::msg::Octomap globalMap;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    std::shared_ptr<std_msgs::msg::Float32> heat_signature_angle;  //declaration
    std::shared_ptr<std_msgs::msg::Bool> something_detected; //declaration
    
    std::shared_ptr<std::vector<std::vector<point3d>>> lidar_frontiers;
    std::shared_ptr<std::vector<std::vector<point3d>>> thermal_frontiers;


    void heat_signature_callback(const std_msgs::msg::Float32 &msg)
    {
        heat_signature_angle->data = msg.data;  // Update the shared pointer with the new angle value
        RCLCPP_INFO(this->get_logger(), "Received: %f", heat_signature_angle->data);
    }

    void detection_callback(const std_msgs::msg::Bool &msg)
    {
        something_detected->data = msg.data;  // Update the shared pointer with the new angle value
        RCLCPP_INFO(this->get_logger(), "Received: %f", something_detected->data);
    }


    void map_callback_octo(const octomap_msgs::msg::Octomap &msg) const
    {
      globalMap_octo=msg;
      octomap::AbstractOcTree *abstract_tree = octomap_msgs::binaryMsgToMap(msg);
      globalMap_octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(abstract_tree));
      if (globalMap_octree==nullptr){
        RCLCPP_ERROR(this->get_logger(), "Octomap is null!");
      }
      RCLCPP_INFO(this->get_logger(), "Map subscribed successfully.");
      mapChanged_octo = true;   
    }
    // void map_callback(const nav_msgs::msg::OccupancyGrid & msg) const
    // {
    //   globalMap = msg;
    //   std::cout << "map subscribed\n";
    //   mapChanged = true;
    // }

    void path_callback(const std_msgs::msg::Bool &msg)
    {
      auto message = nav_msgs::msg::Path();
      // message.data = "Hello, world! " + std::to_string(count_++);
      if (mapChanged_octo && msg.data){
        RCLCPP_INFO(this->get_logger(), "Publishing: ");
        
        // publish obstacle marker
        // obstacle_.header.stamp = this->get_clock()->now();
        // obstacle_pub_->publish(obstacle_);


        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        geometry_msgs::msg::TransformStamped ThermalToWorldTf;

        try {
            if (!this->buffer_->canTransform(
                    "world", "simple_drone/laser_frame",
                    tf2::TimePointZero)) {
                throw "Failed";
            }
            
            RCLCPP_INFO(this->get_logger(), "Can transform");

            sensorToWorldTf = this->buffer_->lookupTransform(
                "world","simple_drone/laser_frame",
                tf2::TimePointZero);

            if (!this->buffer_->canTransform(
                    "world", "simple_drone/front_cam_link",
                    tf2::TimePointZero)) {
                throw "Failed";
            }
            
            RCLCPP_INFO(this->get_logger(), "Can transform");

            ThermalToWorldTf = this->buffer_->lookupTransform(
                "world","simple_drone/front_cam_link",
                tf2::TimePointZero);
        } 
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }

        point3d lidar_origin;
        point3d startstate;
        lidar_origin = point3d(sensorToWorldTf.transform.translation.x,sensorToWorldTf.transform.translation.y ,sensorToWorldTf.transform.translation.z);

        startstate=lidar_origin;

        RCLCPP_INFO(this->get_logger(), "Start state: (%f, %f, %f)", startstate.x(), startstate.y(), startstate.z());


        point3d thermal_origin;
        thermal_origin = point3d(ThermalToWorldTf.transform.translation.x,ThermalToWorldTf.transform.translation.y ,ThermalToWorldTf.transform.translation.z);


        //initializing frontiers vector
        vector<vector<point3d>> frontiers;
        vector<point3d> selected_frontiers;
        vector<pair<point3d, point3d>> candidate_view_points;
        point3d selected_candidate;

        if (something_detected->data){

            frontiers = extractThermalFrontierPoints(); 
            publishThermalMarkers(frontiers);
            vector<double> scores(frontiers.size());
            int max_id=0;
            for(int i=0;i<frontiers.size();i++){
                scores[i]=eval_thermal_score(frontiers[i],ThermalToWorldTf);
                if(scores[i]>scores[max_id]){
                    max_id=i;
                }
            }

            selected_frontiers=frontiers[max_id]; 
            candidate_view_points = extractThermalCandidateViewPoints(selected_frontiers,thermal_origin,5);
            int best_idx = eval_thermal_candidate_yaw_score(candidate_view_points);
            publishThermalCandidateMarkers(candidate_view_points);
            selected_candidate=candidate_view_points[best_idx].first;

        }

        else{
            vector<vector<point3d>> frontiers = extractFrontierPoints(); 
            publishMarkers(frontiers);
            vector<double> scores(frontiers.size());
            int max_id=0;
            for(int i=0;i<frontiers.size();i++){
                scores[i]=eval_score(frontiers[i],sensorToWorldTf);
                if(scores[i]>scores[max_id]){
                    max_id=i;
                }

            }

            selected_frontiers=frontiers[max_id]; 
            candidate_view_points = extractCandidateViewPoints(selected_frontiers,lidar_origin,5);
            publishCandidateMarkers(candidate_view_points);
            selected_candidate=candidate_view_points[0].first;

        }
    
        

        // plan and publish path
        ompl_example_2d::Planner2D planner_(globalMap_octree,selected_candidate,startstate);
        nav_msgs::msg::Path plannedPath;
        // std::shared_ptr<octomap::OcTree> occupancymap(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(globalMap)));
        // if (!occupancymap){
        //   RCLCPP_ERROR(this->get_logger(), "Octomap is null!");
        //   }
        if(planner_.num_pathpublished>0){
          std::cout<<"A new path is found"<<"\n";
          path_changed=planner_.num_pathpublished;
        }
    
        plannedPath = planner_.planPath();
        message = plannedPath;
        publisher_->publish(message);
        std::cout << "Path published.\n";
        // startstate=selected_candidate;


        
    
      }
    }

    void timer_callback()
    {

        try {
            auto tf = buffer_->lookupTransform("world", "simple_drone/front_cam_link", tf2::TimePointZero);
            visualizeYawDifference(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error in yaw debug: %s", ex.what());
        }
    
      if (mapChanged_octo){
        RCLCPP_INFO(this->get_logger(), "Publishing Thermal Octomap: ");
        
        // publish obstacle marker
        // obstacle_.header.stamp = this->get_clock()->now();
        // obstacle_pub_->publish(obstacle_);
        octomap::Pointcloud points;

        geometry_msgs::msg::TransformStamped ThermalToWorldTf;

        try {
            if (!this->buffer_->canTransform(
                    "world", "simple_drone/front_cam_link",
                    tf2::TimePointZero)) {
                throw "Failed";
            }
            
            RCLCPP_INFO(this->get_logger(), "Can transform");

            ThermalToWorldTf = this->buffer_->lookupTransform(
                "world","simple_drone/front_cam_link",
                tf2::TimePointZero);
        } 
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }

        point3d thermal_origin;
        thermal_origin = point3d(ThermalToWorldTf.transform.translation.x,ThermalToWorldTf.transform.translation.y ,ThermalToWorldTf.transform.translation.z);
        points=castSensorRays(globalMap_octree,ThermalToWorldTf);
        thermal_tree->insertPointCloud(points, thermal_origin, thermalsensor.max_range, false, true);

        // vector<vector<point3d>> frontiers = extractThermalFrontierPoints(); 
        // publishMarkers(frontiers);
        // vector<double> scores(frontiers.size());
        // int max_id=0;
        // for(int i=0;i<frontiers.size();i++){
        //     scores[i]=eval_thermal_score(frontiers[i],ThermalToWorldTf);
        //     if(scores[i]>scores[max_id]){
        //         max_id=i;
        //     }
        // }
        // vector<point3d> selected_frontiers=frontiers[max_id];
        // vector<pair<point3d, point3d>> candidate_view_points = extractCandidateViewPoints(selected_frontiers, thermal_origin, 5);
        // // mapChanged = false;
        // std::cout << "Path published.\n";
        // publishCandidateMarkers(candidate_view_points);

        


        }
        publishoccupiedthermal(*thermal_tree);
    }

void publishMarkers(const vector<vector<point3d>>& frontiers) {
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto& cluster : frontiers) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world"; // Set your reference frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "frontiers";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.5; // Size of the points
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Assign a unique color for this cluster
        marker.color.r = static_cast<float>((marker_id * 73) % 255) / 255.0; // Arbitrary multiplier for variation
        marker.color.g = static_cast<float>((marker_id * 43) % 255) / 255.0;
        marker.color.b = static_cast<float>((marker_id * 29) % 255) / 255.0;
        marker.color.a = 1.0; // Fully opaque

        // Convert each point3d to geometry_msgs::msg::Point
        for (const auto& point : cluster) {
            geometry_msgs::msg::Point ros_point;
            ros_point.x = point.x();
            ros_point.y = point.y();
            ros_point.z = point.z();
            marker.points.push_back(ros_point);
        }
        marker_array.markers.push_back(marker);
    }

    // Publish the marker array
    frontiers_pub_->publish(marker_array);
}


void publishThermalMarkers(const vector<vector<point3d>>& frontiers) {
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto& cluster : frontiers) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world"; // Set your reference frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "frontiers";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.5; // Size of the points
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Assign a unique color for this cluster
        marker.color.r = static_cast<float>((marker_id * 73) % 255) / 255.0; // Arbitrary multiplier for variation
        marker.color.g = static_cast<float>((marker_id * 43) % 255) / 255.0;
        marker.color.b = static_cast<float>((marker_id * 29) % 255) / 255.0;
        marker.color.a = 1.0; // Fully opaque

        // Convert each point3d to geometry_msgs::msg::Point
        for (const auto& point : cluster) {
            geometry_msgs::msg::Point ros_point;
            ros_point.x = point.x();
            ros_point.y = point.y();
            ros_point.z = point.z();
            marker.points.push_back(ros_point);
        }
        marker_array.markers.push_back(marker);
    }

    // Publish the marker array
    thermal_frontiers_pub_->publish(marker_array);
}



void publishCandidateMarkers(const std::vector<std::pair<point3d, point3d>> &candidates,int &max_idx)
{
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto &candidate : candidates) {
        const auto &position = candidate.first;
        const auto &orientation = candidate.second;

        // Create a sphere marker for the position
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "candidate_points";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set position
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();

        // Set orientation (not relevant for spheres but can be set to identity)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale
        marker.scale.x = 0.5;  // Diameter of the sphere
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Set color
        if((id-1)==max_idx){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

        }
        marker_array.markers.push_back(marker);

        // Create an arrow marker for orientation
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = rclcpp::Clock().now();
        arrow.ns = "candidate_orientations";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        // Set position (start of arrow at the sphere position)
        arrow.pose.position.x = position.x();
        arrow.pose.position.y = position.y();
        arrow.pose.position.z = position.z();

        // Set orientation (yaw angle from orientation.z)
        double yaw = orientation.z();
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = sin(yaw / 2.0);
        arrow.pose.orientation.w = cos(yaw / 2.0);

        // Set scale (length and width of arrow)
        arrow.scale.x = 0.5;  // Arrow shaft length
        arrow.scale.y = 0.05; // Arrow shaft diameter
        arrow.scale.z = 0.05; // Arrow head diameter

        // Set color
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 0.8;  // Transparency

        marker_array.markers.push_back(arrow);
    }

    // Publish the marker array
    candidates_pub_->publish(marker_array);
}

//Overloading the function for the publishing of the candidate marker array for debugging control
void publishCandidateMarkers(const std::vector<std::pair<point3d, point3d>> &candidates)
{
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto &candidate : candidates) {
        const auto &position = candidate.first;
        const auto &orientation = candidate.second;

        // Create a sphere marker for the position
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "candidate_points";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set position
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();

        // Set orientation (not relevant for spheres but can be set to identity)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale
        marker.scale.x = 0.5;  // Diameter of the sphere
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        if((id-1)==0){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        }

        marker_array.markers.push_back(marker);

        // Create an arrow marker for orientation
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = rclcpp::Clock().now();
        arrow.ns = "candidate_orientations";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        // Set position (start of arrow at the sphere position)
        arrow.pose.position.x = position.x();
        arrow.pose.position.y = position.y();
        arrow.pose.position.z = position.z();

        // Set orientation (yaw angle from orientation.z)
        double yaw = orientation.z();
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = sin(yaw / 2.0);
        arrow.pose.orientation.w = cos(yaw / 2.0);

        // Set scale (length and width of arrow)
        arrow.scale.x = 0.5;  // Arrow shaft length
        arrow.scale.y = 0.05; // Arrow shaft diameter
        arrow.scale.z = 0.05; // Arrow head diameter

        // Set color
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 0.8;  // Transparency

        marker_array.markers.push_back(arrow);
    }

    // Publish the marker array
    candidates_pub_->publish(marker_array);
}

void publishThermalCandidateMarkers(const std::vector<std::pair<point3d, point3d>> &candidates)
{
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto &candidate : candidates) {
        const auto &position = candidate.first;
        const auto &orientation = candidate.second;

        // Create a sphere marker for the position
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "thermal_candidate_points";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set position
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();

        // Set orientation (not relevant for spheres but can be set to identity)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale
        marker.scale.x = 0.5;  // Diameter of the sphere
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        if((id-1)==0){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

        }
        else{
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        }

        marker_array.markers.push_back(marker);

        // Create an arrow marker for orientation
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = rclcpp::Clock().now();
        arrow.ns = "thermal_candidate_orientations";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        // Set position (start of arrow at the sphere position)
        arrow.pose.position.x = position.x();
        arrow.pose.position.y = position.y();
        arrow.pose.position.z = position.z();

        // Set orientation (yaw angle from orientation.z)
        double yaw = orientation.z();
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = sin(yaw / 2.0);
        arrow.pose.orientation.w = cos(yaw / 2.0);

        // Set scale (length and width of arrow)
        arrow.scale.x = 0.5;  // Arrow shaft length
        arrow.scale.y = 0.05; // Arrow shaft diameter
        arrow.scale.z = 0.05; // Arrow head diameter

        // Set color
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 0.8;  // Transparency

        marker_array.markers.push_back(arrow);
    }

    // Publish the marker array
    thermal_candidates_pub_->publish(marker_array);
}






//This is the actual working code for the frontier points and I am modifying this for a bounding box query
// so that we can save on the compute time it takes to find the friontiers, as in the current implementation
//  it would take alot of computation to search through the entire map uselessly.
    vector<vector<point3d>> extractFrontierPoints() {

      vector<vector<point3d>> frontier_groups;
      vector<point3d> frontier_points;
      octomap::OcTreeNode *n_cur_frontier;
      bool frontier_true;         // whether or not a frontier point
      bool belong_old;            //whether or not belong to old group
      double distance;
      double R1 = 3;            //group size
      double x_cur, y_cur, z_cur;
      double octo_reso=0.9;


      for(octomap::OcTree::leaf_iterator n = globalMap_octree->begin_leafs(globalMap_octree->getTreeDepth()); n != globalMap_octree->end_leafs(); ++n)
      {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

        if(!globalMap_octree->isNodeOccupied(*n))
          {
          x_cur = n.getX();
          y_cur = n.getY();
          z_cur = n.getZ();

          if(z_cur < octo_reso)    continue;
          if(z_cur > 0.2 + octo_reso+2.1)    continue;
          //if there are unknown around the cube, the cube is frontier
          for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
              for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
              {
                  n_cur_frontier = globalMap_octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                  if(!n_cur_frontier)
                  {
                      frontier_true = true;
                      continue;            
                  }

              }
              if(frontier_true)
              {
                  // divede frontier points into groups
                  if(frontier_groups.size() < 1)
                  {
                      frontier_points.resize(1);
                      frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                      frontier_groups.push_back(frontier_points);
                      frontier_points.clear();
                  }
                  else
                  {
                      bool belong_old = false;            

                      for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                              distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)+pow(frontier_groups[u][0].z()-z_cur, 2)) ;
                              if(distance < R1){
                                frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                                belong_old = true;
                                break;
                              }
                      }
                      if(!belong_old){
                                frontier_points.resize(1);
                                frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                                frontier_groups.push_back(frontier_points);
                                frontier_points.clear();
                      }                              
                  }
              } 
          }
      }
    return frontier_groups;
  }

    vector<vector<point3d>> extractThermalFrontierPoints() {

      vector<vector<point3d>> frontier_groups;
      vector<point3d> frontier_points;
      octomap::OcTreeNode *n_cur_frontier;
      bool frontier_true;         // whether or not a frontier point
      bool belong_old;            //whether or not belong to old group
      double distance;
      double R1 = 3;            //group size
      double x_cur, y_cur, z_cur;
      double octo_reso=0.9;


      for(octomap::OcTree::leaf_iterator n = thermal_tree->begin_leafs(thermal_tree->getTreeDepth()); n != thermal_tree->end_leafs(); ++n)
      {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

        if(!thermal_tree->isNodeOccupied(*n))
          {
          x_cur = n.getX();
          y_cur = n.getY();
          z_cur = n.getZ();

          if(z_cur < octo_reso)    continue;
          if(z_cur > 0.2 + octo_reso+2.1)    continue;
          //if there are unknown around the cube, the cube is frontier
          for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
              for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
              {
                  n_cur_frontier = thermal_tree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                  if(!n_cur_frontier)
                  {
                      frontier_true = true;
                      continue;            
                  }

              }
              if(frontier_true)
              {
                  // divede frontier points into groups
                  if(frontier_groups.size() < 1)
                  {
                      frontier_points.resize(1);
                      frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                      frontier_groups.push_back(frontier_points);
                      frontier_points.clear();
                  }
                  else
                  {
                      bool belong_old = false;            

                      for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                              distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)+pow(frontier_groups[u][0].z()-z_cur, 2)) ;
                              if(distance < R1){
                                frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                                belong_old = true;
                                break;
                              }
                      }
                      if(!belong_old){
                                frontier_points.resize(1);
                                frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                                frontier_groups.push_back(frontier_points);
                                frontier_points.clear();
                      }                              
                  }
              } 
          }
      }
    return frontier_groups;
  }



    vector<vector<point3d>> extractFrontierPoints(point3d sensor_orig) {

      vector<vector<point3d>> frontier_groups;
      vector<point3d> frontier_points;
      octomap::OcTreeNode *n_cur_frontier;
      bool frontier_true;         // whether or not a frontier point
      bool belong_old;            //whether or not belong to old group
      double distance;
      double R1 = 3;            //group size
      double x_cur, y_cur, z_cur;
      double octo_reso=0.9;

    //   point3d min_point=(sensor_orig.x()-20,sensor_orig.y()-20,0.0);
    //   point3d max_point=(sensor_orig.x()+20,sensor_orig.y()+20,5.0);
    point3d min_point(sensor_orig.x() - 20, sensor_orig.y() - 20, 0.0);
    point3d max_point(sensor_orig.x() + 20, sensor_orig.y() + 20, 10.0);


      for(octomap::OcTree::leaf_bbx_iterator n = globalMap_octree->begin_leafs_bbx(min_point,max_point),end=globalMap_octree->end_leafs_bbx(); n!=end; ++n)
      {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

        if(!globalMap_octree->isNodeOccupied(*n))
          {
          x_cur = n.getX();
          y_cur = n.getY();
          z_cur = n.getZ();

          if(z_cur < octo_reso)    continue;
          if(z_cur > 0.2 + octo_reso+2.1)    continue;
          //if there are unknown around the cube, the cube is frontier
          for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
              for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
              {
                  n_cur_frontier = globalMap_octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                  if(!n_cur_frontier)
                  {
                      frontier_true = true;
                      continue;            
                  }

              }
              if(frontier_true)
              {
                  // divede frontier points into groups
                  if(frontier_groups.size() < 1)
                  {
                      frontier_points.resize(1);
                      frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                      frontier_groups.push_back(frontier_points);
                      frontier_points.clear();
                  }
                  else
                  {
                      bool belong_old = false;            

                      for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                              distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)+pow(frontier_groups[u][0].z()-z_cur, 2)) ;
                              if(distance < R1){
                                frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                                belong_old = true;
                                break;
                              }
                      }
                      if(!belong_old){
                                frontier_points.resize(1);
                                frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                                frontier_groups.push_back(frontier_points);
                                frontier_points.clear();
                      }                              
                  }
              } 
          }
      }
    return frontier_groups;
  }
// vector<vector<point3d>> extractFrontierPoints() {
//     vector<vector<point3d>> frontier_groups;
//     double R1 = 2;            // Group size
//     double octo_reso = 0.5;

//     // Parallelize the iteration over octree leaf nodes
//     #pragma omp parallel
//     {
//         // Thread-local storage for temporary groups
//         vector<vector<point3d>> local_frontier_groups;
//         vector<point3d> frontier_points;

//         #pragma omp for schedule(dynamic, 10) nowait
//         for (octomap::OcTree::leaf_iterator n = globalMap_octree->begin_leafs(globalMap_octree->getTreeDepth()); 
//              n != globalMap_octree->end_leafs(); ++n) {

//             if (!globalMap_octree->isNodeOccupied(*n)) {
//                 double x_cur = n.getX();
//                 double y_cur = n.getY();
//                 double z_cur = n.getZ();

//                 if (z_cur < octo_reso || z_cur > 0.4 + octo_reso + 2.0)
//                     continue;

//                 // Check if it's a frontier point
//                 bool frontier_true = false;
//                 for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso) {
//                     for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso) {
//                         octomap::OcTreeNode *n_cur_frontier = globalMap_octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
//                         if (!n_cur_frontier) {
//                             frontier_true = true;
//                             break;
//                         }
//                     }
//                     if (frontier_true)
//                         break;
//                 }

//                 if (frontier_true) {
//                     // Divide frontier points into groups
//                     bool belong_old = false;

//                     for (size_t u = 0; u < local_frontier_groups.size(); u++) {
//                         double distance = sqrt(pow(local_frontier_groups[u][0].x() - x_cur, 2) + 
//                                                pow(local_frontier_groups[u][0].y() - y_cur, 2)+pow(local_frontier_groups[u][0].z() - z_cur, 2));
//                         if (distance < R1) {
//                             local_frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
//                             belong_old = true;
//                             break;
//                         }
//                     }

//                     if (!belong_old) {
//                         frontier_points.push_back(point3d(x_cur, y_cur, z_cur));
//                         local_frontier_groups.push_back(frontier_points);
//                         frontier_points.clear();
//                     }
//                 }
//             }
//         }

//         // Combine local groups into the global group
//         #pragma omp critical
//         {
//             for (const auto &group : local_frontier_groups) {
//                 frontier_groups.push_back(group);
//             }
//         }
//     }

//     return frontier_groups;
// }
// Overloading for all clusters of frontiers
  vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<vector<point3d>> frontier_groups, point3d sensor_orig, int n ) {
      double R2_min = 1.0;        // distance from candidate view point to frontier centers, in meters.
      double R2_max = 4.0;
      double R3 = 0.3;        // to other frontiers

      octomap::OcTreeNode *n_cur_3d;
      vector<pair<point3d, point3d>> candidates;
      // double z = sensor_orig.z();
      double x, y,z, yaw, distance_can;
      const double PI = 3.1415926;
      const double octo_reso = 0.5;

          for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++) {
              for(double yaw = 0; yaw < 2*PI; yaw += PI*2/n )
                  for(double R2 = R2_min; R2<=R2_max; R2+=0.5) { 
                  x = frontier_groups[u][0].x() - R2 * cos(yaw);
                  y = frontier_groups[u][0].y() - R2 * sin(yaw);
                  z = frontier_groups[u][0].z();

                  bool candidate_valid = true;
                  n_cur_3d = globalMap_octree->search(point3d(x, y, z));


                  if (!n_cur_3d) {
                      candidate_valid = false;
                      continue;
                  }

                  if(sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2) + pow(z-sensor_orig.z(),2)) < 0.25){
                    candidate_valid = false;// delete candidates close to sensor_orig
                    continue;
                  }

                  else{

                      // check candidate to other frontiers;
                      for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++)
                          for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                              distance_can = sqrt(pow(x - frontier_groups[n][m].x(),2) + pow(y - frontier_groups[n][m].y(),2)+pow(z - frontier_groups[n][m].z(),2));
                              if(distance_can < R3){
                                  candidate_valid = false;        //delete candidates close to frontier
                                  continue;
                              }
                      }
                  
                      // volumn check
                      for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                          for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                              for (double z_buf = z-0.3; z_buf <z+0.3; z_buf += octo_reso)
                              {
                                  n_cur_3d = globalMap_octree->search(point3d(x_buf, y_buf, z_buf));
                                  if(!n_cur_3d)       continue;
                                  else if (globalMap_octree->isNodeOccupied(n_cur_3d)){
                                  candidate_valid = false;//delete candidates which have ccupied cubes around in 3D area
                                  continue;
                                  }  
                              }

                  }

                  if (candidate_valid)
                  {
                      candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                  }
              }
          }
      return candidates;
  }


//This is the overloading done for evaulation of single cluster
vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<point3d> frontier_groups, point3d sensor_orig, int n) {
    double R2_min = 1.0;        // Distance from candidate view point to frontier centers
    double R2_max = 4.0;
    double R3 = 0.3;            // To other frontiers
    const double PI = 3.1415926;
    const double octo_reso = 0.5;

    // Shared vector for candidates
    vector<pair<point3d, point3d>> candidates;

    // Parallelize the outer loop over frontier groups
    #pragma omp parallel
    {
        // Thread-local storage for temporary candidates
        vector<pair<point3d, point3d>> local_candidates;

        #pragma omp for schedule(dynamic, 1)
        for (double yaw = 0; yaw < 2 * PI; yaw += PI * 2 / n) {
            for (double R2 = R2_min; R2 <= R2_max; R2 += 1) {
                double x = frontier_groups[0].x() - R2 * cos(yaw);
                double y = frontier_groups[0].y() - R2 * sin(yaw);
                double z = frontier_groups[0].z();

                bool candidate_valid = true;
                octomap::OcTreeNode *n_cur_3d = globalMap_octree->search(point3d(x, y, z));

                if (!n_cur_3d) {
                    candidate_valid = false;
                    continue;
                }

                if (sqrt(pow(x - sensor_orig.x(), 2) + pow(y - sensor_orig.y(), 2) + pow(z - sensor_orig.z(), 2)) < 0.25) {
                    candidate_valid = false; // Delete candidates close to sensor_orig
                    continue;
                }

                // // Check candidate against other frontiers
                // for (size_t n = 0; n < frontier_groups.size(); n++) {
                //     for (size_t m = 0; m < frontier_groups[n].size(); m++) {
                //         double distance_can = sqrt(pow(x - frontier_groups[n][m].x(), 2) + 
                //                                     pow(y - frontier_groups[n][m].y(), 2) + 
                //                                     pow(z - frontier_groups[n][m].z(), 2));
                //         if (distance_can < R3) {
                //             candidate_valid = false; // Delete candidates close to frontier
                //             break;
                //         }
                //     }
                //     if (!candidate_valid)
                //         break;
                // }

                // Volume check
                for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) {
                    for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso) {
                        for (double z_buf = z - 0.3; z_buf < z + 0.3; z_buf += octo_reso) {
                            n_cur_3d = globalMap_octree->search(point3d(x_buf, y_buf, z_buf));
                            if (!n_cur_3d)
                                continue;
                            if (globalMap_octree->isNodeOccupied(n_cur_3d)) {
                                candidate_valid = false; // Delete candidates near occupied cubes
                                break;
                            }
                        }
                        if (!candidate_valid)
                            break;
                    }
                    if (!candidate_valid)
                        break;
                }

                if (candidate_valid) {
                    // Add valid candidate to the thread-local storage
                    local_candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
            }
        }

        // Merge thread-local candidates into the global vector
        #pragma omp critical
        {
            candidates.insert(candidates.end(), local_candidates.begin(), local_candidates.end());
        }
    }

    return candidates;
}

vector<pair<point3d, point3d>> extractThermalCandidateViewPoints(vector<point3d> frontier_groups, point3d sensor_orig, int n) {
    double R2_min = 1.0;        // Distance from candidate view point to frontier centers
    double R2_max = 4.0;
    double R3 = 0.3;            // To other frontiers
    const double PI = 3.1415926;
    const double octo_reso = 0.9;

    // Shared vector for candidates
    vector<pair<point3d, point3d>> candidates;

    // Parallelize the outer loop over frontier groups
    #pragma omp parallel
    {
        // Thread-local storage for temporary candidates
        vector<pair<point3d, point3d>> local_candidates;

        #pragma omp for schedule(dynamic, 1)
        for (double yaw = 0; yaw < 2 * PI; yaw += PI * 2 / n) {
            for (double R2 = R2_min; R2 <= R2_max; R2 += 1) {
                double x = frontier_groups[0].x() - R2 * cos(yaw);
                double y = frontier_groups[0].y() - R2 * sin(yaw);
                double z = frontier_groups[0].z();

                bool candidate_valid = true;
                octomap::OcTreeNode *n_cur_3d = thermal_tree->search(point3d(x, y, z));

                if (!n_cur_3d) {
                    candidate_valid = false;
                    continue;
                }

                if (sqrt(pow(x - sensor_orig.x(), 2) + pow(y - sensor_orig.y(), 2) + pow(z - sensor_orig.z(), 2)) < 0.25) {
                    candidate_valid = false; // Delete candidates close to sensor_orig
                    continue;
                }

                // Volume check
                for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) {
                    for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso) {
                        for (double z_buf = z - 0.3; z_buf < z + 0.3; z_buf += octo_reso) {
                            n_cur_3d = thermal_tree->search(point3d(x_buf, y_buf, z_buf));
                            if (!n_cur_3d)
                                continue;
                            if (thermal_tree->isNodeOccupied(n_cur_3d)) {
                                candidate_valid = false; // Delete candidates near occupied cubes
                                break;
                            }
                        }
                        if (!candidate_valid)
                            break;
                    }
                    if (!candidate_valid)
                        break;
                }

                if (candidate_valid) {
                    // Add valid candidate to the thread-local storage
                    local_candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
            }
        }

        // Merge thread-local candidates into the global vector
        #pragma omp critical
        {
            candidates.insert(candidates.end(), local_candidates.begin(), local_candidates.end());
        }
    }

    return candidates;
}




double countFreeVolume(const std::shared_ptr<octomap::OcTree> octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}
// volume counter with bbx_iterator
double countFreeVolume(const std::shared_ptr<octomap::OcTree> octree,point3d sensor_orig) {
    double volume = 0;
    point3d min_point(sensor_orig.x() - 20, sensor_orig.y() - 20, 0.0);
    point3d max_point(sensor_orig.x() + 20, sensor_orig.y() + 20, 10.0);

    for(octomap::OcTree::leaf_bbx_iterator n = globalMap_octree->begin_leafs_bbx(min_point,max_point),end=globalMap_octree->end_leafs_bbx(); n!=end; ++n){
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}
octomap::Pointcloud castSensorRays(const std::shared_ptr<octomap::OcTree> octree, const point3d &position,
                                 const point3d &sensor_orientation) {
    octomap::Pointcloud hits;

    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(lidarsensor.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    #pragma omp parallel for
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, lidarsensor.max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i).normalized() * lidarsensor.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

octomap::Pointcloud castSensorRays(const std::shared_ptr<octomap::OcTree> octree,const geometry_msgs::msg::TransformStamped ThermalToWorldTf) {
    octomap::Pointcloud hits;
    octomap::Pointcloud RaysToCast;

    double w=ThermalToWorldTf.transform.rotation.w;
    double x=ThermalToWorldTf.transform.rotation.x;
    double y=ThermalToWorldTf.transform.rotation.y;
    double z=ThermalToWorldTf.transform.rotation.z;

    tf2::Quaternion quaternion(x,y,z,w);
    tf2::Matrix3x3 rotation_matrix(quaternion);

    double roll,pitch,yaw;
    rotation_matrix.getRPY(roll,pitch,yaw);



    RaysToCast.push_back(thermalsensor.SensorRays);
    RaysToCast.rotate(roll,pitch,yaw);
    point3d end;

    point3d position(ThermalToWorldTf.transform.translation.x,ThermalToWorldTf.transform.translation.y,ThermalToWorldTf.transform.translation.z);


    #pragma omp parallel for
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, thermalsensor.max_range)) {
            hits.push_back(end);
        }
        // else {
        //     end = RaysToCast.getPoint(i) * thermalsensor.max_range;
        //     end += position;
        //     hits.push_back(end);
        // }
    }
    return hits;
}

double calc_MI(const std::shared_ptr<octomap::OcTree> octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = std::make_shared<octomap::OcTree>(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, lidarsensor.max_range, true, true);
    double after = countFreeVolume(octree_copy);
    return after - before;
}
vector<int> sort_MIs(const vector<double> &v){
    vector<int> idx(v.size());
    iota(idx.begin(), idx.end(),0);

    sort(idx.begin(), idx.end(), 
        [&v](int i1, int i2) {return v[i1] > v[i2];});

    return idx;
}

double eval_score(vector<point3d> frontier,const geometry_msgs::msg::TransformStamped sensorToWorldTf){
    double size=frontier.size();

    double w=sensorToWorldTf.transform.rotation.w;
    double x=sensorToWorldTf.transform.rotation.x;
    double y=sensorToWorldTf.transform.rotation.y;
    double z=sensorToWorldTf.transform.rotation.z;

    tf2::Quaternion quaternion(x,y,z,w);
    tf2::Matrix3x3 rotation_matrix(quaternion);

    double roll,pitch,yaw;
    rotation_matrix.getRPY(roll,pitch,yaw);

    double x_pos=sensorToWorldTf.transform.translation.x;
    double y_pos=sensorToWorldTf.transform.translation.y;
    double desired_yaw=atan2(y_pos,x_pos);

    if(desired_yaw<0){
        desired_yaw=desired_yaw+2*PI;
    }
    if(yaw<0){
        yaw=yaw+2*PI;
    }

    double diff_yaw=abs(yaw-desired_yaw);

    double x_pos_des=frontier[0].x();
    double y_pos_des=frontier[0].y();

    double distance=sqrt(pow(x_pos-x_pos_des,2)+pow(y_pos-y_pos_des,2));

    double score=5*size-10*diff_yaw+2*distance;
    return(score);
}

double eval_thermal_score(const std::vector<point3d>& frontier, const geometry_msgs::msg::TransformStamped& tf) {
    if (frontier.empty()) return -M_PI;  // Worst case

    // Compute centroid of frontier
    double x_sum = 0, y_sum = 0;
    for (const auto& pt : frontier) {
        x_sum += pt.x();
        y_sum += pt.y();
    }
    double x_avg = x_sum / frontier.size();
    double y_avg = y_sum / frontier.size();

    // Get drone pose
    double x_drone = tf.transform.translation.x;
    double y_drone = tf.transform.translation.y;

    // Get drone yaw
    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
    double roll, pitch, drone_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, drone_yaw);
    drone_yaw = fmod(drone_yaw + 2 * PI, 2 * PI);

    // Compute yaw to frontier in world frame
    double frontier_yaw = atan2(y_avg - y_drone, x_avg - x_drone);
    frontier_yaw = fmod(frontier_yaw + 2 * PI, 2 * PI);

    // Compute yaw to frontier in drone frame
    double relative_frontier_yaw = frontier_yaw - drone_yaw;
    relative_frontier_yaw = fmod(relative_frontier_yaw + 2 * PI, 2 * PI);

    // Normalize desired_yaw and compare
    // double desired_yaw = fmod(heat_signature_angle->data + 2 * PI, 2 * PI);
    double desired_yaw = heat_signature_angle->data;
    desired_yaw = desired_yaw * M_PI / 180.0;  //converting to radians

    double diff_yaw = fabs(relative_frontier_yaw - desired_yaw);
    if (diff_yaw > PI) diff_yaw = 2 * PI - diff_yaw;

    // visualizeYawDifference(tf, point3d(x_avg, y_avg, 0.0), desired_yaw, diff_yaw);

    

    return (1/diff_yaw);  // Higher score for smaller yaw difference
}

void visualizeYawDifference(const geometry_msgs::msg::TransformStamped& tf) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    point3d drone_pos(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);

    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
    double roll, pitch, drone_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, drone_yaw);

    double arrow_len = 3.0;

    auto make_arrow = [&](double angle, float r, float g, float b, const std::string& ns) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = rclcpp::Clock().now();
        arrow.ns = ns;
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x = drone_pos.x();
        start.y = drone_pos.y();
        start.z = drone_pos.z();

        end.x = start.x + arrow_len * cos(angle);
        end.y = start.y + arrow_len * sin(angle);

        end.z = start.z;

        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow.scale.x = 0.2;
        arrow.scale.y = 0.4;
        arrow.scale.z = 0.4;
        arrow.color.r = r;
        arrow.color.g = g;
        arrow.color.b = b;
        arrow.color.a = 1.0;

        return arrow;
    };

    // Arrows
// Convert from degrees to radians
    double heat_angle_deg = heat_signature_angle->data;
    double heat_angle_drone = heat_angle_deg * M_PI / 180.0;

    // Normalize to [0, 2π)
    heat_angle_drone = fmod(heat_angle_drone + 2 * PI, 2 * PI);

    // Get world-frame heat angle
    double heat_angle_world = fmod(drone_yaw + heat_angle_drone, 2 * PI);

    


    // double frontier_yaw_world = atan2(frontier_center.y() - drone_pos.y(), frontier_center.x() - drone_pos.x());

// Normalize to [0, 2π]
// heat_angle_world = fmod(heat_angle_world + 2 * PI, 2 * PI);
// frontier_yaw_world = fmod(frontier_yaw_world + 2 * PI, 2 * PI);

marker_array.markers.push_back(make_arrow(drone_yaw, 1.0, 1.0, 1.0, "drone_heading"));         // white
marker_array.markers.push_back(make_arrow(heat_angle_world, 1.0, 0.0, 0.0, "heat_direction")); // red
// text_marker.text = "Heat Yaw: " + std::to_string(heat_angle_world * 180.0 / M_PI) + "°";


// marker_array.markers.push_back(make_arrow(frontier_yaw_world, 0.0, 1.0, 0.0, "frontier_direction")); // green

    // Text
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = rclcpp::Clock().now();
    text_marker.ns = "angle_diff_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = drone_pos.x();
    text_marker.pose.position.y = drone_pos.y();
    text_marker.pose.position.z = drone_pos.z() + 2.0;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    // text_marker.text = "Yaw Diff: " + std::to_string(diff_yaw * 180.0 / M_PI) + "°";

    marker_array.markers.push_back(text_marker);

    yaw_debug_pub_->publish(marker_array);
}



//overloaded visualizer function that takes input of a frontier too
void visualizeYawDifference(const geometry_msgs::msg::TransformStamped& tf, const point3d& frontier_center, double desired_yaw_rad, double diff_yaw_rad) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    point3d drone_pos(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);

    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
    double roll, pitch, drone_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, drone_yaw);

    double arrow_len = 3.0;

    auto make_arrow = [&](double angle, float r, float g, float b, const std::string& ns) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = rclcpp::Clock().now();
        arrow.ns = ns;
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x = drone_pos.x();
        start.y = drone_pos.y();
        start.z = drone_pos.z();

        end.x = start.x + arrow_len * cos(angle);
        end.y = start.y + arrow_len * sin(angle);
        end.z = start.z;

        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow.scale.x = 0.2;
        arrow.scale.y = 0.4;
        arrow.scale.z = 0.4;
        arrow.color.r = r;
        arrow.color.g = g;
        arrow.color.b = b;
        arrow.color.a = 1.0;

        return arrow;
    };

    // Convert desired yaw (in degrees) to world angle
    double heat_angle_world = fmod(drone_yaw + desired_yaw_rad, 2 * PI);

    // Compute frontier yaw in world frame
    double frontier_yaw_world = atan2(frontier_center.y() - drone_pos.y(), frontier_center.x() - drone_pos.x());
    frontier_yaw_world = fmod(frontier_yaw_world + 2 * PI, 2 * PI);

    // Arrows
    marker_array.markers.push_back(make_arrow(drone_yaw, 1.0, 1.0, 1.0, "drone_heading"));         // white
    marker_array.markers.push_back(make_arrow(heat_angle_world, 1.0, 0.0, 0.0, "heat_direction")); // red
    marker_array.markers.push_back(make_arrow(frontier_yaw_world, 0.0, 1.0, 0.0, "frontier_direction")); // green

    // Text
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = rclcpp::Clock().now();
    text_marker.ns = "angle_diff_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = drone_pos.x();
    text_marker.pose.position.y = drone_pos.y();
    text_marker.pose.position.z = drone_pos.z() + 2.0;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    text_marker.text = "Yaw Diff: " + std::to_string(diff_yaw_rad * 180.0 / M_PI) + "°";

    marker_array.markers.push_back(text_marker);

    yaw_debug_pub_->publish(marker_array);
}


void visualizeYawDifference(const geometry_msgs::msg::TransformStamped& tf,
    const std::pair<point3d, point3d>& candidate,
    double desired_yaw_rad, double diff_yaw_rad_placeholder = 0.0) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    point3d drone_pos(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    point3d candidate_pos = candidate.first;

    // Get drone yaw
    tf2::Quaternion q(
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z,
    tf.transform.rotation.w);
    double roll, pitch, drone_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, drone_yaw);
    drone_yaw = fmod(drone_yaw + 2 * PI, 2 * PI);

    // Compute yaw to candidate (in world frame)
    double candidate_yaw_world = atan2(candidate_pos.y() - drone_pos.y(), candidate_pos.x() - drone_pos.x());
    candidate_yaw_world = fmod(candidate_yaw_world + 2 * PI, 2 * PI);

    // Convert to drone frame
    double candidate_yaw_relative = fmod(candidate_yaw_world - drone_yaw + 2 * PI, 2 * PI);

    // Normalize desired yaw
    desired_yaw_rad = fmod(desired_yaw_rad + 2 * PI, 2 * PI);

    // Compute difference
    double diff_yaw = fabs(candidate_yaw_relative - desired_yaw_rad);
    if (diff_yaw > PI) diff_yaw = 2 * PI - diff_yaw;

    // Create arrows
    double arrow_len = 3.0;

    auto make_arrow = [&](double angle, float r, float g, float b, const std::string& ns) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = rclcpp::Clock().now();
    arrow.ns = ns;
    arrow.id = id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = drone_pos.x();
    start.y = drone_pos.y();
    start.z = drone_pos.z();

    end.x = start.x + arrow_len * cos(angle);
    end.y = start.y + arrow_len * sin(angle);
    end.z = start.z;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    arrow.scale.x = 0.2;
    arrow.scale.y = 0.4;
    arrow.scale.z = 0.4;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    return arrow;
};

    marker_array.markers.push_back(make_arrow(drone_yaw, 1.0, 1.0, 1.0, "drone_heading"));             // white
    marker_array.markers.push_back(make_arrow(drone_yaw + desired_yaw_rad, 1.0, 0.0, 0.0, "heat"));    // red
    marker_array.markers.push_back(make_arrow(candidate_yaw_world, 0.0, 1.0, 0.0, "candidate"));       // green

    // Yaw diff text
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = rclcpp::Clock().now();
    text_marker.ns = "angle_diff_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = drone_pos.x();
    text_marker.pose.position.y = drone_pos.y();
    text_marker.pose.position.z = drone_pos.z() + 2.0;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    text_marker.text = "Yaw Diff: " + std::to_string(diff_yaw * 180.0 / M_PI) + "°";
    marker_array.markers.push_back(text_marker);

    yaw_debug_pub_->publish(marker_array);
}

int eval_thermal_candidate_yaw_score(const std::vector<std::pair<point3d, point3d>> &candidates) {
    // Convert desired yaw from degrees to radians
    double desired_yaw = heat_signature_angle->data * M_PI / 180.0;

    // Normalize to [0, 2π]
    desired_yaw = fmod(desired_yaw + 2 * PI, 2 * PI);

    int best_idx = -1;
    double best_score = 0;

    for (size_t i = 0; i < candidates.size(); ++i) {
        double candidate_yaw = fmod(candidates[i].second.z() + 2 * PI, 2 * PI);

        double diff_yaw = fabs(candidate_yaw - desired_yaw);
        if (diff_yaw > PI) diff_yaw = 2 * PI - diff_yaw;

        double score = 1/diff_yaw;

        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}


void publishoccupiedthermal(const octomap::OcTree tree){
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world"; // Set your reference frame
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "frontiers";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.5; // Size of the points
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 1.0; 
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    double x_cur, y_cur, z_cur; 


    for(octomap::OcTree::leaf_iterator n = tree.begin_leafs(tree.getTreeDepth()); n != tree.end_leafs(); ++n){
        if(tree.isNodeOccupied(*n)){
            x_cur = n.getX();
            y_cur = n.getY();
            z_cur = n.getZ();
            geometry_msgs::msg::Point ros_point;
            ros_point.x = x_cur;
            ros_point.y = y_cur;
            ros_point.z = z_cur;
            marker.points.push_back(ros_point);
        }
    }
    marker_array.markers.push_back(marker);
    thermal_pub_->publish(marker_array);
}


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thermal_candidates_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thermal_frontiers_pub_;    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thermal_pub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_octo;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_request_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heat_signature_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr something_detected_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr yaw_debug_pub_;

    visualization_msgs::msg::Marker obstacle_;
};



// void thread_publisher(void){

//     rclcpp::spin(std::make_shared<PathPublisher>());

// }







int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    // std::thread thread_pub = std::thread(thread_publisher);
    // thread_pub.join();
    rclcpp::shutdown();
    return 0;
}


    // for (const auto& cluster : frontiers) {
    //     visualization_msgs::msg::Marker marker;
    //     marker.header.frame_id = "world"; // Set your reference frame
    //     marker.header.stamp = rclcpp::Clock().now();
    //     marker.ns = "frontiers";
    //     marker.id = marker_id++;
    //     marker.type = visualization_msgs::msg::Marker::POINTS;
    //     marker.action = visualization_msgs::msg::Marker::ADD;

    //     marker.scale.x = 0.5; // Size of the points
    //     marker.scale.y = 0.5;
    //     marker.scale.z = 0.5;

    //     // Assign a unique color for this cluster
    //     marker.color.r = static_cast<float>((marker_id * 73) % 255) / 255.0; // Arbitrary multiplier for variation
    //     marker.color.g = static_cast<float>((marker_id * 43) % 255) / 255.0;
    //     marker.color.b = static_cast<float>((marker_id * 29) % 255) / 255.0;
    //     marker.color.a = 1.0; // Fully opaque

    //     // Convert each point3d to geometry_msgs::msg::Point
    //     for (const auto& point : cluster) {
    //         geometry_msgs::msg::Point ros_point;
    //         ros_point.x = point.x();
    //         ros_point.y = point.y();
    //         ros_point.z = point.z();
    //         marker.points.push_back(ros_point);
    //     }
    //     marker_array.markers.push_back(marker);
    // }
