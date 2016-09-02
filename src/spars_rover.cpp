// author: Shaun Howard (smh150@case.edu)
// SPARS Rover Planner for Orange Century Robot (uses SPARS2 planner)

#include <spars_rover/rover.h>

void SPARSRover::initialize(std::string name, costmap_2d::Costmap2DROS* new_costmap_ros) {
    if (!initialized) {
        costmap_ros = new_costmap_ros;
        costmap = costmap_ros->getCostmap();
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size, costmap->getResolution());
        private_nh.param("min_dist_from_robot", robot_radius, 0.2);
        world_model = new base_local_planner::CostmapModel(*costmap);
        plan_pub = private_nh.advertise<nav_msgs::Path>("SPARSRover_plan", 1, true);
        initialized = true;
        ROS_WARN("Initialized SPARS Rover planner!");
    }
    else {
        ROS_WARN("Planner is ready to go. Nothing to initialize.");
    }
}

void SPARSRover::mapCallback(nav_msgs::OccupancyGrid latest_map){
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

void SPARSRover::amclPoseCallback(geometry_msgs::PoseWithCovarianceStamped new_pose) {
    amcl_pose = new_pose.pose.pose;
    pose_found = true;
}

void SPARSRover::initializeSubscribers() {
    map_sub = nh.subscribe("map", 1, &SPARSRover::mapCallback, this);
    amcl_pose_sub = nh.subscribe("amcl_pose", 1, &SparsRover::amclPoseCallback, this);
}

bool SPARSRover::stateIsValid(const ompl::base::State *state) {
    const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
    geometry_msgs::Point point;
    point.x = se2state->getX();
    point.y = se2state->getY();
    std::vector<geometry_msgs::Point> footprint;
    footprint.push_back(point);
    double point_cost = world_model->footprintCost(point, footprint, robot_radius, robot_radius);
    return point_cost >= 0;
}

bool SPARSRover::makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
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

//   tf::Pose pose;
//    tf::poseMsgToTF(turtle_start.pose, pose);
//    double start_yaw = tf::getYaw(pose.getRotation());
    ompl::base::ScopedState<> start_state(space);
    start_state->as<ompl::base::SE2StateSpace::StateType>()->setX(start.pose.position.x);
    start_state->as<ompl::base::SE2StateSpace::StateType>()->setY(start.pose.position.y);   
//    start_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start_yaw);
    ROS_DEBUG_STREAM("Set planner start state to (" << start.pose.position.x << ", " << start.pose.position.y << ")");

    //tf::poseMsgToTF(turtle_goal.pose, pose);
    //double goal_yaw = tf::getYaw(pose.getRotation());
    ompl::base::ScopedState<> goal_state(space);
    goal_state->as<ompl::base::SE2StateSpace::StateType>()->setX(goal.pose.position.x);
    goal_state->as<ompl::base::SE2StateSpace::StateType>()->setY(goal.pose.position.y);
    //goal_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal_yaw);
    ROS_DEBUG_STREAM("Set planner goal state to (" << goal.pose.position.x << ", " << goal.pose.position.y << ")");
    ss.setStartAndGoalStates(start_state, goal_state);

    ompl::base::PlannerPtr spars2(new ompl::geometric::SPARStwo(si));
    ss.setPlanner(spars2);

    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    ss.setOptimizationObjective(obj);

    // do not use this unless benchmark planner disabled
    ss.setStateValidityChecker(boost::bind(&SPARSRover::stateIsValid, this, _1));

    ss.setup();

    ss.print();

    ompl::base::PlannerStatus solved = ss.solve(2);
    if (solved) {
        ROS_INFO("Found SPARS2 solution:");
        ss.simplifySolution();
        ompl::geometric::PathGeometric planner_soln = ss.getSolutionPath();
        planner_soln.print(std::cout);
        nav_msgs::Path soln = extractNavPath(planner_soln);
        geometry_msgs::PoseStamped temp_pose;
        for (unsigned i = 0; i < soln.poses.size(); i++) {
            temp_pose.header.frame_id = costmap_ros->getGlobalFrameID();
            temp_pose.pose = soln.poses[i].pose;
            ros::Time plan_time = ros::Time(0);
            temp_pose.header.stamp = plan_time;
            plan.push_back(temp_pose);
        }
        plan_pub.publish(soln);
        ros::spinOnce();
    } else {
        ROS_WARN("Could not find SPARS2 solution...");
    }
return true;
}
