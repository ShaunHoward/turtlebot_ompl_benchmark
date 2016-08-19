#ifndef TURTLEBOT_RRT_RRT_H
#define TURTLEBOT_RRT_RRT_H

#include <ros/ros.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>


class RRT : public nav_core::BaseGlobalPlanner {
public:
    RRT();
    RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
    nav_msgs::OccupancyGrid::Ptr getInflatedMap();

private:
    ros::NodeHandle nh;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid::Ptr inflated_map;
    geometry_msgs::Pose amcl_pose;

    ros::Subscriber map_sub;
    ros::Subscriber amcl_pose_sub;
    ros::Publisher plan_pub;

    costmap_2d::Costmap2DROS* costmap_ros;
    costmap_2d::Costmap2D* costmap;
    base_local_planner::WorldModel* world_model;

    bool unknown_okay;
    bool initialized;

    int occupied_threshold;

    double min_x, max_x;
    double min_y, max_y;
    double robot_radius;
    double step_size;
    
    bool map_found;
    bool pose_found;

    void initializeSubscribers();
    void mapCallback(nav_msgs::OccupancyGrid new_map);
    void amclPoseCallback(geometry_msgs::PoseWithCovarianceStamped new_pose);
    bool stateIsValid(const ompl::base::State *state);
    nav_msgs::Path extractNavPath(ompl::geometric::PathGeometric ompl_path);
};

geometry_msgs::Quaternion convertPlanarPhiToQuaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

RRT::RRT() :
        costmap_ros(NULL),
        map_found(false),
        pose_found(false),
        initialized(false),
        robot_radius(0.2),
        unknown_okay(false),
        occupied_threshold(50),
        max_x(std::numeric_limits<int>::min()),
        max_y(std::numeric_limits<int>::min()),
        min_x(std::numeric_limits<int>::max()),
        min_y(std::numeric_limits<int>::max()) {
    ROS_INFO("Initializing RRT for Turtlebot");
    initializeSubscribers();
    ROS_INFO("RRT not constructed with costmap");
}

RRT::RRT(std::string name, costmap_2d::Costmap2DROS* new_costmap_ros) :
        costmap_ros(NULL),
        map_found(false),
        pose_found(false),
        initialized(false),
        unknown_okay(false),
        occupied_threshold(50),
        max_x(std::numeric_limits<int>::min()),
        max_y(std::numeric_limits<int>::min()),
        min_x(std::numeric_limits<int>::max()),
        min_y(std::numeric_limits<int>::max()) {
    ROS_INFO("Initializing RRT for Turtlebot");
    initializeSubscribers();
    initialize(name, new_costmap_ros);
    ROS_INFO("RRT ready with costmap");
}

nav_msgs::OccupancyGrid::Ptr RRT::getInflatedMap(){
    return inflated_map;
}

nav_msgs::Path RRT::extractNavPath(ompl::geometric::PathGeometric ompl_path) {
    nav_msgs::Path ros_path;
    ros_path.header.frame_id = "map";
    std::vector<ompl::base::State*> ompl_states = ompl_path.getStates();
    for (unsigned i = 0; i < ompl_states.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = ompl_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getX();
        pose.pose.position.y = ompl_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getY();
        double yaw = ompl_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
        pose.pose.orientation = convertPlanarPhiToQuaternion(yaw);
        ros_path.poses.push_back(pose);
    }
    return ros_path;
}

/* Register in navstack */
PLUGINLIB_EXPORT_CLASS(RRT, nav_core::BaseGlobalPlanner)

#endif //TURTLEBOT_RRT_RRT_H