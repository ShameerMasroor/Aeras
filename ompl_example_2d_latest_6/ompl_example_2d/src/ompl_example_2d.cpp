/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "ompl_example_2d/ompl_example_2d.hpp"
#include <rclcpp/rclcpp.hpp>

// STL
#include <string>
#include <math.h>
#include <cmath>  // for std::isfinite
#include <limits>

using namespace std;


namespace ompl_example_2d {

/// occupancy map used for planning
// nav_msgs::msg::OccupancyGrid occupancyMap;
// octomap_msgs::msg::Octomap occupancyMap;
// octomap::OcTree occupancyMap;
std::shared_ptr<octomap::OcTree> occupancyMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);

// Planner2D::Planner2D(void){
//     std::cout<<"planner 2D started\n";
//     configure(double minX, double maxX , double minY , double maxY, double minZ, double maxZ);
// }

Planner2D::Planner2D(const std::shared_ptr<octomap::OcTree> globalMap_octree, point3d goalstate,point3d startstate, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_input)
{
    std::cout << "planner 2D started\n";
    occupancyMap = globalMap_octree;
    kdtree = kdtree_input;
    if(occupancyMap==nullptr){
        // std::cout<<"Occupany Map is a nullptr"<<"\n";
    }
    else{
        // std::cout<<"Occupancy Map is okay"<<"\n";
    }
        // // Extract bounds from the OctoMap
    double minX, minY, minZ, maxX, maxY, maxZ;
    occupancyMap->getMetricMin(minX, minY, minZ);
    occupancyMap->getMetricMax(maxX, maxY, maxZ);

    // std::cout<<"Value of minX is: "<<minX<<"\n";
    // std::cout<<"Value of maxX is: "<<maxX<<"\n";

    // std::cout<<"Value of minY is: "<<minY<<"\n";
    // std::cout<<"Value of maxY is: "<<maxY<<"\n";
    
    // std::cout<<"Value of minZ is: "<<minZ<<"\n";
    // std::cout<<"Value of maxZ is: "<<maxZ<<"\n";

    configure(minX,maxX,minY,maxY,minZ,maxZ,goalstate,startstate);
}

Planner2D::~Planner2D()
{
}

/// check if the current state is valid
// bool isStateValid(const ob::State *state){
//     // get x coord of the robot
//     const auto *coordX =
//             state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//     // get y coord of the robot
//     const auto *coordY =
//             state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

//     //! Comment this part of the code if you'd like to use occupancy grid
//     // define the obstacle
//     if (coordX->values[0]<5.1&&coordX->values[0]>5.0){
//         if (coordY->values[0]<4.0&&coordY->values[0]>-5.0){
//             return false;
//         }
//     }
//     //! Comment this part of the code if you'd like to use occupancy grid

//     //! Your code goes below
//     // Hint: uncoment the code below:
// //    std::cout << "occupancyMap.info.origin.position " << occupancyMap.info.origin.position.x <<
// //                 ", " << occupancyMap.info.origin.position.y << "\n";
// //    std::cout << "occupancyMap.info.resolution " << occupancyMap.info.resolution << "\n";
// //    std::cout << "occupancyMap.info.width " << occupancyMap.info.width << "\n";
// //    std::cout << "occupancyMap.info.height " << occupancyMap.info.height << "\n";
//     //! Your code goes above
//     return true;
// }

// bool isStateValid(const ob::State *state) {
//     // Get x and y coordinates of the robot
//     const auto *coordX =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//     const auto *coordY =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

//     double x = coordX->values[0];
//     double y = coordY->values[0];

//     // Extract occupancy map information
//     double origin_x = occupancyMap.info.origin.position.x;
//     double origin_y = occupancyMap.info.origin.position.y;
//     double resolution = occupancyMap.info.resolution;
//     int width = occupancyMap.info.width;
//     int height = occupancyMap.info.height;

//     // Compute grid cell indices from world coordinates
//     int grid_x = static_cast<int>((x - origin_x) / resolution);
//     int grid_y = static_cast<int>((y - origin_y) / resolution);

//     // Check if indices are within map bounds
//     if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
//         return false; // Outside the map bounds, invalid state
//     }

//     // Get the index in the occupancy grid data
//     int index = grid_y * width + grid_x;

//     // Check the occupancy value
//     if (occupancyMap.data[index] > 0 || occupancyMap.data[index] == -1 ) {
//         return false; // Occupied cell, invalid state
//     }

//     return true; // Valid state
// }

// This is my actual checker, for the octomap.
// bool Planner2D::isStateValid(const ob::State *state) {
//     // Get x, y, and z coordinates of the drone
//     const auto *coordX =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//     const auto *coordY =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
//     const auto *coordZ =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

//     double x = coordX->values[0];
//     double y = coordY->values[0];
//     double z = coordZ->values[0];

//     double minX, minY, minZ, maxX, maxY, maxZ;
//     occupancyMap->getMetricMin(minX, minY, minZ);
//     occupancyMap->getMetricMax(maxX, maxY, maxZ);

//     if (x < minX || x > maxX || y < minY || y > maxY || z < minZ || z > maxZ) {
//         // std::cout<<"The node is outside bounds"<<"\n";
//         return false; // Outside bounds
//     }

//     // Query occupancy state in OctoMap
//     octomap::OcTreeNode* node = occupancyMap->search(x, y, z);

//     if (node == nullptr) {
//         std::cout<<"The node is unknown"<<"\n";
//         return false; // Unknown space
        
//     }
//     if (occupancyMap->isNodeOccupied(node)) {
//         std::cout<<"The node is occupied"<<"\n";
//         return false; // Occupied space
//     }

//     return true; // Valid state
// }

bool Planner2D::isStateValid(const ob::State *state) {
    // Get x, y, and z coordinates of the drone
    const auto *coordX =
        state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *coordY =
        state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const auto *coordZ =
        state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    double x = coordX->values[0];
    double y = coordY->values[0];
    double z = coordZ->values[0];

    

    vector<int> indices;
    vector<float> distances;
    double collision_radius = 0.5;
    pcl::PointXYZ query(x, y, z);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        std::cerr << "Query point has invalid (NaN or Inf) values!" << std::endl;
        return false;
    }

    if (kdtree.radiusSearch(query, collision_radius, indices, distances) > 0){
        return (false);
    }
    return (true);
}

// bool Planner2D::isStateValid(const ob::State *state) {
//     // Get x, y, and z coordinates of the drone
//     const auto *coordX =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//     const auto *coordY =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
//     const auto *coordZ =
//         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

//     double x = coordX->values[0];
//     double y = coordY->values[0];
//     double z = coordZ->values[0];

//     double minX, minY, minZ, maxX, maxY, maxZ;
//     // std::shared_ptr<octomap::OcTree> occupancymap(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(occupancyMap)));
//     // if (!occupancymap) // Always check for null before using pointers
//     // {
//     //     RCLCPP_ERROR(this->get_logger(), "Octomap is null!");
//     //     // return;
//     // }
//     occupancyMap->getMetricMin(minX, minY, minZ);
//     occupancyMap->getMetricMax(maxX, maxY, maxZ);

//     if (x < minX || x > maxX || y < minY || y > maxY || z < minZ || z > maxZ) {
//         // std::cout<<"The node is outside bounds"<<"\n";
//         return false; // Outside bounds
//     }

//     // Query occupancy state in OctoMap
//     // octomap::OcTreeNode* node = occupancyMap->search(x, y, z);
//     // point3d min_point, max_point;
//     // point3d min_point();
//     // point3d max_point();
//     // if(z==0.0){
//     //     point3d min_point(x - 0.25, y - 0.25, z);
//     //     point3d max_point(x + 0.25, y + 0.25, z+0.25);
//     // }
//     // else{
//     //     point3d min_point(x - 0.25, y - 0.25, z-0.25);
//     //     point3d max_point(x + 0.25, y + 0.25, z+0.25);
//     // }

//     point3d min_point(x - 0.25, y - 0.25, z);
//     point3d max_point(x + 0.25, y + 0.25, z+0.25);


//     for(octomap::OcTree::leaf_bbx_iterator n = occupancyMap->begin_leafs_bbx(min_point,max_point),end=occupancyMap->end_leafs_bbx(); n!=end; ++n){
//         // if (n == nullptr) {
//         // // std::cout<<"The node is unknown"<<"\n";
//         //     return false; // Unknown space
        
//         // }
        
//         if (occupancyMap->isNodeOccupied(*n)) {
//         // std::cout<<"The node is occupied"<<"\n";
//             return false; // Occupied space
//         }
//     }
//     return true; // Valid state
// }

// bool Planner2D::isStateValid(const ob::State *state) {
//     // Get x, y, and z coordinates of the drone
//     // const auto *coordX =
//     //     state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//     // const auto *coordY =
//     //     state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
//     // const auto *coordZ =
//     //     state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

//     // double x = coordX->values[0];
//     // double y = coordY->values[0];
//     // double z = coordZ->values[0];

//     // double minX, minY, minZ, maxX, maxY, maxZ;
//     // std::shared_ptr<octomap::OcTree> occupancymap(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(occupancyMap)));
//     // if (!occupancymap) // Always check for null before using pointers
//     // {
//     //     RCLCPP_ERROR(this->get_logger(), "Octomap is null!");
//     //     // return;
//     // }
//     // occupancymap->getMetricMin(minX, minY, minZ);
//     // occupancymap->getMetricMax(maxX, maxY, maxZ);

//     // if (x < minX || x > maxX || y < minY || y > maxY || z < minZ || z > maxZ) {
//     //     return false; // Outside bounds
//     // }

//     // Query occupancy state in OctoMap
//     // octomap::OcTreeNode* node = occupancymap->search(x, y, z);
//     // if (node == nullptr) {
//     //     return false; // Unknown space
//     // }
//     // if (occupancymap->isNodeOccupied(node)) {
//     //     return false; // Occupied space
//     // }

//     return true; // Valid state
// }

/// extract path
nav_msgs::msg::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){
    nav_msgs::msg::Path plannedPath;
    plannedPath.header.frame_id = "/world";
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();

    auto spaceInformation = pdef->getSpaceInformation();
    og::PathSimplifier pathSimplifier(spaceInformation);
    // print the path to screen
    // path->print(std::cout);
    // convert to geometric path

    auto *path_ = path.get()->as<og::PathGeometric>();

    if (path_) {
        // Smooth the path using B-Spline smoothing
        // pathSimplifier.smoothBSpline(*path_, 3);
        pathSimplifier.simplifyMax(*path_);
        // pathSimplifier.smoothBSpline(*path_, 5); //may need to disable this
    }

    // iterate over each position
    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        const ob::State* state = path_->getState(i);
        // get x coord of the robot
        const auto *coordX =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // get y coord of the robot
        const auto *coordY =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        const auto *coordZ =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
        // fill in the ROS PoseStamped structure...
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordX->values[0];
        poseMsg.pose.position.y = coordY->values[0];
        poseMsg.pose.position.z = coordZ->values[0];
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/world";
        poseMsg.header.stamp = rclcpp::Clock().now();
        // ... and add the pose to the path
        plannedPath.poses.push_back(poseMsg);
    }
    std::cout << "planned path size: " << plannedPath.poses.size() << "\n";
    return plannedPath;
}

/*!
 * plan path
 */
nav_msgs::msg::Path Planner2D::planPath(void){

    // occupancyMap = std::make_shared<octomap::OcTree>(globalMap);

    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    // define state checking callback
    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.001);

    // problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start.get(), *goal.get());

    ob::OptimizationObjectivePtr objective(new ob::PathLengthOptimizationObjective(si));
    pdef->setOptimizationObjective(objective);

    // create planner
    auto planner(std::make_shared<og::RRTConnect>(si));
    // configure the planner
    planner->setRange(maxStepLength);// max step length
    planner->setProblemDefinition(pdef);
    planner->setup();

    // solve motion planning problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(50);

    nav_msgs::msg::Path plannedPath;
    if (solved) {// if cussess
        // get the planned path
        plannedPath=extractPath(pdef.get());
        num_pathpublished++;
    }
    std::cout << "path planned\n";
    return plannedPath;
}

/// configure planner
void Planner2D::configure(double minX, double maxX , double minY , double maxY, double minZ, double maxZ, point3d goalstate,point3d startstate){
    dim = 3;//2D problem
    maxStepLength = 1;// max step length

    
    // create bounds for the x axis
    // coordXBound.reset(new ob::RealVectorBounds(1));
    coordXBound = std::make_shared<ob::RealVectorBounds>(1);
    // coordXBound->setLow(-13.0);
    // coordXBound->setHigh(13.0);
    coordXBound->setLow(minX);
    coordXBound->setHigh(maxX);

    // create bounds for the y axis
    coordYBound = std::make_shared<ob::RealVectorBounds>(1);
    // coordYBound->setLow(-13.0);
    // coordYBound->setHigh(13.0);
    coordYBound->setLow(minY);
    coordYBound->setHigh(maxY);

    // Create bounds for the z-axis
    coordZBound = std::make_shared<ob::RealVectorBounds>(1);
    coordZBound->setLow(-0.5);
    coordZBound->setHigh(3);
    // coordZBound->setLow(minZ);
    // coordZBound->setHigh(maxZ);

    // // construct the state space we are planning in
    // auto coordX(std::make_shared<ob::RealVectorStateSpace>(1));
    // auto coordY(std::make_shared<ob::RealVectorStateSpace>(1));
    // auto coordZ(std::make_shared<ob::RealVectorStateSpace>(1));
    // // auto coordX = std::make_shared<ob::RealVectorStateSpace>(1);
    // // auto coordY = std::make_shared<ob::RealVectorStateSpace>(1);
    // // auto coordZ = std::make_shared<ob::RealVectorStateSpace>(1);
    // space = coordX +coordY + coordZ;

    auto coordX = std::make_shared<ob::RealVectorStateSpace>(1);
    auto coordY = std::make_shared<ob::RealVectorStateSpace>(1);
    auto coordZ = std::make_shared<ob::RealVectorStateSpace>(1);

    space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(coordX, 1.0);
    space->addSubspace(coordY, 1.0);
    space->addSubspace(coordZ, 1.0);
    space->lock();

    // create bounds for the x axis
    coordX->setBounds(*coordXBound.get());

    // create bounds for the y axis
    coordY->setBounds(*coordYBound.get());

    // create bounds for the z axis
    coordZ->setBounds(*coordZBound.get());

//     // define the start position
//     start.reset(new ob::ScopedState<>(space));
//     (*start.get())[0]=0.0;
//     (*start.get())[1]=-2.5;
//     (*start.get())[2]=0.0;
// //    start.get()->random();
//     // define the goal position
//     goal.reset(new ob::ScopedState<>(space));
//     (*goal.get())[0]=12.0;
//     (*goal.get())[1]=-4.0;
//     (*goal.get())[2]=4.0;
// //    goal.get()->random();


    // Define the start position (example values; adjust as needed)
    start.reset(new ob::ScopedState<>(space));
    (*start.get())[0] = startstate.x(); // Slightly offset from the map edge
    (*start.get())[1] = startstate.y();
    (*start.get())[2] = startstate.z();


    // Define the goal position (example values; adjust as needed)
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0] = goalstate.x(); // Slightly offset from the map edge
    (*goal.get())[1] = goalstate.y();
    (*goal.get())[2] = goalstate.z();

}

// void Planner2D::configure(void) {
//     dim = 3; // 3D problem
//     maxStepLength = 0.1; // Max step length
//     std::shared_ptr<octomap::OcTree> occupancymap(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(occupancyMap)));
//     // if (!occupancymap){ // Always check for null before using pointers{
//     //     RCLCPP_ERROR(this->get_logger(), "Octomap is null!");
//     //     return;
//     // }
//     // Extract bounds from the OctoMap
//     double minX, minY, minZ, maxX, maxY, maxZ;
//     occupancymap->getMetricMin(minX, minY, minZ);
//     occupancymap->getMetricMax(maxX, maxY, maxZ);
//     // Create bounds for the x-axis
//     coordXBound.reset(new ob::RealVectorBounds(1));
//     coordXBound->setLow(minX);
//     coordXBound->setHigh(maxX);
//     // Create bounds for the y-axis
//     coordYBound.reset(new ob::RealVectorBounds(1));
//     coordYBound->setLow(minY);
//     coordYBound->setHigh(maxY);
//     // Create bounds for the z-axis
//     coordZBound.reset(new ob::RealVectorBounds(1));
//     coordZBound->setLow(minZ);
//     coordZBound->setHigh(maxZ);
//     // Construct the state space we are planning in
//     auto coordX = std::make_shared<ob::RealVectorStateSpace>(1);
//     auto coordY = std::make_shared<ob::RealVectorStateSpace>(1);
//     auto coordZ = std::make_shared<ob::RealVectorStateSpace>(1);
//     space = coordX + coordY + coordZ;
//     // Set bounds for each axis
//     coordX->setBounds(*coordXBound.get());
//     coordY->setBounds(*coordYBound.get());
//     coordZ->setBounds(*coordZBound.get());
//     // Define the start position (example values; adjust as needed)
//     start.reset(new ob::ScopedState<>(space));
//     (*start.get())[0] = 0.0; // Slightly offset from the map edge
//     (*start.get())[1] = 0.0;
//     (*start.get())[2] = 0.0;
//     // Define the goal position (example values; adjust as needed)
//     goal.reset(new ob::ScopedState<>(space));
//     (*goal.get())[0] = -2.0; // Slightly offset from the map edge
//     (*goal.get())[1] = -3.0;
//     (*goal.get())[2] = 0.0;
// }


} /* namespace */

