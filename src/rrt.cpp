#include <turtlebot_rrt/rrt.h>

void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* new_costmap_ros) {
    if (!initialized) {
        costmap_ros = new_costmap_ros;
        costmap = costmap_ros->getCostmap();
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size, costmap->getResolution());
        private_nh.param("min_dist_from_robot", robot_radius, 0.2);
        world_model = new base_local_planner::CostmapModel(*costmap);

        plan_pub = private_nh.advertise<nav_msgs::Path>("rrt_plan", 1, true);
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
        if (bounds.points[i].x < min_x) {
            min_x = bounds.points[i].x;
        } else if (bounds.points[i].x > max_x) {
            max_x = bounds.points[i].x;
        }

        if (bounds.points[i].y < min_y) {
            min_y = bounds.points[i].y;
        } else if (bounds.points[i].y > max_y) {
            max_y = bounds.points[i].y;
        }
    }
}

void RRT::amclPoseCallback(geometry_msgs::PoseWithCovarianceStamped new_pose) {
    amcl_pose = new_pose.pose.pose;
    pose_found = true;
}

void RRT::initializeSubscribers() {
	map_sub = nh.subscribe("map", 1, &RRT::mapCallback, this);
	amcl_pose_sub = nh.subscribe("amcl_pose", 1, &RRT::amclPoseCallback, this);
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

bool RRT::makePlan(const geometry_msgs::PoseStamped& turtle_start,
                  const geometry_msgs::PoseStamped& turtle_goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) {

    if (!initialized) {
        ROS_ERROR("initialize() must be called on the planner before using... exiting...");
        return false;
    }

    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    int x = 0, y = 1;
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(x, costmap->getOriginX());
    bounds.setHigh(x, costmap->getSizeInMetersX() - costmap->getOriginX());
    bounds.setLow(y, costmap->getOriginY());
    bounds.setHigh(y, costmap->getSizeInMetersY() - costmap->getOriginY());
    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    ompl::geometric::SimpleSetup ss(si);

    ompl::base::PlannerPtr rrt_star(new ompl::geometric::RRTstar(si));
    ss.setPlanner(rrt_star);

    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    ss.setOptimizationObjective(obj);

    ss.setStateValidityChecker(boost::bind(&RRT::stateIsValid, this, _1));

    tf::Pose pose;
    tf::poseMsgToTF(turtle_start.pose, pose);
    double start_yaw = tf::getYaw(pose.getRotation());
    ompl::base::ScopedState<> start_state(space);
    start_state->as<ompl::base::SE2StateSpace::StateType>()->setX(turtle_start.pose.position.x);
    start_state->as<ompl::base::SE2StateSpace::StateType>()->setY(turtle_start.pose.position.y);   
    start_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start_yaw);
    ROS_DEBUG_STREAM("Set rrt start state to ( " << turtle_start.pose.position.x << ", " << turtle_start.pose.position.y << ", " << start_yaw << ")");

    tf::poseMsgToTF(turtle_goal.pose, pose);
    double goal_yaw = tf::getYaw(pose.getRotation());
    ompl::base::ScopedState<> goal_state(space);
    goal_state->as<ompl::base::SE2StateSpace::StateType>()->setX(turtle_goal.pose.position.x);
    goal_state->as<ompl::base::SE2StateSpace::StateType>()->setY(turtle_goal.pose.position.y);
    goal_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal_yaw);
    ROS_DEBUG_STREAM("Set rrt goal state to ( " << turtle_goal.pose.position.x << ", " << turtle_goal.pose.position.y << ", " << goal_yaw << ")");
    ss.setStartAndGoalStates(start_state, goal_state);

    ss.setup();
    ss.print();

    ompl::base::PlannerStatus solved = ss.solve(10.0);
    if (solved) {
        ROS_INFO("Found RRT solution:");
        ss.simplifySolution();
        ompl::geometric::PathGeometric planner_soln = ss.getSolutionPath();
        planner_soln.print(std::cout);
        nav_msgs::Path turtle_soln = extractNavPath(planner_soln);
        geometry_msgs::PoseStamped temp_pose;
        for (unsigned i = 0; i < turtle_soln.poses.size(); i++) {
            ros::Time plan_time = ros::Time::now();
            temp_pose.header.stamp = plan_time;
            temp_pose.header.frame_id = costmap_ros->getGlobalFrameID();
            temp_pose.pose = turtle_soln.poses[i].pose;
            plan.push_back(temp_pose);
        }
        plan_pub.publish(turtle_soln);
        ros::spinOnce();
    } else {
        ROS_WARN("Could not find RRT solution...");
    }
return true;
}