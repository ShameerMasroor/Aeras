/*
 * ompl_example_2d.hpp
 *
 *  Created on: April 6, 2020
 *  Updated on: April 8, 2023
 *      Author: Dominik Belter
 *	 Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#pragma once

// ROS
#include <geometry_msgs/msg/pose_stamped.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>


// #include <moveit/ompl_interface/ompl_interface.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.6/ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRT.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl-1.6/ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef octomap::point3d point3d;

namespace ompl_example_2d {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:

    /*!
   * Constructor.
   */
    // Planner2D(void);
    // Planner2D(const std::shared_ptr<octomap::OcTree> globalMap_octree, point3d goalstate,point3d startstate);
    Planner2D(const std::shared_ptr<octomap::OcTree> globalMap_octree, point3d goalstate,point3d startstate, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_input);

    /*!
   * Destructor.
   */
    virtual ~Planner2D();

    /*!
   * plan path
   */
    nav_msgs::msg::Path planPath(void);
    int num_pathpublished=0;
    // nav_msgs::msg::Path planPath(const octomap_msgs::msg::Octomap& globalMap);
    // nav_msgs::msg::Path planPath(const octomap_msgs::msg::Octomap& globalMap);
    // nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& globalMap);

private:

    /// problem dim
    int dim;

    static bool isStateValid(const ob::State *state);

    /// max step length
    double maxStepLength;

    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// bounds for the z axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordZBound;

    /// start position
    std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
    std::shared_ptr<ompl::base::CompoundStateSpace> space;

    /// configure node
    // void configure(void);
    void configure(double minX, double maxX , double minY , double maxY, double minZ, double maxZ, point3d goalstate,point3d startstate);

    /// extract path
    nav_msgs::msg::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
