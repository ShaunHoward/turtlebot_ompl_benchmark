#include <turtlebot_rrt_benchmark/rrt.h>

void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* new_costmap_ros) {
    if (!initialized) {
        costmap_ros = new_costmap_ros;
        costmap = costmap_ros->getCostmap();
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size, costmap->getResolution());
        private_nh.param("min_dist_from_robot", robot_radius, 0.2);
        world_model = new base_local_planner::CostmapModel(*costmap);

        plan_publisher = private_nh.advertise<nav_msgs::Path>("rrt_plan", 1, true);
        initialized = true;
    }
    else {
        ROS_WARN("Planner is ready to go. Nothing to initialize.");
    }
}

void RRT::mapCallback(nav_msgs::OccupancyGrid latest_map){
	if (map_found) {
        ROS_WARN("Map updates found without previous updates applied...");
    }

    map = latest_map;
    inflated_map = occupancy_grid_utils::inflateObstacles(map, robot_radius, unknown_okay);
    map_found = true;

    geometry_msgs::Polygon bounds = occupancy_grid_utils::gridPolygon(inflated_map->info);
    // determine map limits
    for (unsigned i = 0; i < bounds.points.size(); i++) {
        if (bounds.points[i].x > max_x) {
            max_x = bounds.points[i].x;
        }
        if (bounds.points[i].x < min_x) {
            min_x = bounds.points[i].x;
        }
        if (bounds.points[i].y > max_y) {
            max_y = bounds.points[i].y;
        }
        if (bounds.points[i].y < min_y) {
            min_y = bounds.points[i].y;
        }
    }
}

void RRT::amclPoseCallback(geometry_msgs::PoseWithCovarianceStamped new_pose) {
    amcl_pose = new_pose.pose.pose;
    pose_received = true;
}

void RRT::initializeSubscribers() {
	map_sub = nh.subscribe("map", 1, &RRT::mapCallback, this);
	amcl_pose_subscriber = nh.subscribe("amcl_pose", 1, &RRT::amclPoseCallback, this);
}

bool RRT::stateIsValid(const ompl::base::State *state) {
	const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
	geometry_msgs::Point point;
    point.x = se2state->getX();
    point.y = se2state->getY();
    std::vector<geometry_msgs::Point> footprint;
    footprint.push_back(point);
    double point_cost = world_model->footprintCost(point, footprint, robot_radius, robot_radius);
    return point_cost >= 0;
}