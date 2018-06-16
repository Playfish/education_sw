#include <pluginlib/class_list_macros.h>

#include <vfh_local_planner/vfh_planner_ros.h>
#include <base_local_planner/goal_functions.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(vfh_local_planner::VFHPlannerROS, nav_core::BaseLocalPlanner)

namespace vfh_local_planner {

  void VFHPlannerROS::reconfigureCB(VFHPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }
#if 0
      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_trans_vel = config.max_trans_vel;
      limits.min_trans_vel = config.min_trans_vel;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_rot_vel = config.max_rot_vel;
      limits.min_rot_vel = config.min_rot_vel;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_limit_trans = config.acc_limit_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.rot_stopped_vel = config.rot_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
#endif
  }

  //start initialize
  void VFHPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      ROS_INFO("Initializing VFH");

      costmap_ros_ = costmap_ros;
      tf_ = tf;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();

      ros::NodeHandle nh_private_("~/" + name);

      g_plan_pub_ =  nh_private_.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ =  nh_private_.advertise<nav_msgs::Path>("local_plan", 1);
      // For me
      m_cell_size = 100;          // mm, cell dimension
      m_window_diameter = 60;     // number of cells
      m_sector_angle = 5;         // deg, sector angle

      // VFH Parameters
      // mm, double, safe distance at 0 m/s
      if (!nh_private_.getParam ("m_safety_dist_0ms", m_safety_dist_0ms))
        m_safety_dist_0ms = 100;

      // mm, double, safe distance at 1 m/s
      if (!nh_private_.getParam ("m_safety_dist_1ms", m_safety_dist_1ms))
        m_safety_dist_1ms = 100;

      // mm/sec, int, max speed
      if (!nh_private_.getParam ("m_max_speed", m_max_speed))
        m_max_speed= 200;

      // mm/sec, int, max speed in the narrow opening
      if (!nh_private_.getParam ("m_max_speed_narrow_opening", m_max_speed_narrow_opening))
        m_max_speed_narrow_opening= 200;

      // mm/sec, int, max speed in the wide opening
      if (!nh_private_.getParam ("m_max_speed_wide_opening", m_max_speed_wide_opening))
        m_max_speed_wide_opening= 300;

      // mm/sec^2, int, max acceleration
      if (!nh_private_.getParam ("m_max_acceleration", m_max_acceleration))
        m_max_acceleration = 200;

      // deg/sec, int, min turn rate <--- not used
      if (!nh_private_.getParam ("m_min_turnrate", m_min_turnrate))
        m_min_turnrate = 40;

      // deg/sec, int, max turn rate at 0 m/s
      if (!nh_private_.getParam ("m_max_turnrate_0ms", m_max_turnrate_0ms))
        m_max_turnrate_0ms = 40;

      // deg/sec, int, max turn rate at 1 m/s
      if (!nh_private_.getParam ("m_max_turnrate_1ms", m_max_turnrate_1ms))
        m_max_turnrate_1ms = 40;

      m_min_turn_radius_safety_factor = 1.0;         // double

      // double, low threshold free space at 0 m/s
      if (!nh_private_.getParam ("m_free_space_cutoff_0ms", m_free_space_cutoff_0ms))
        m_free_space_cutoff_0ms = 2000000.0;

      // double, high threshold obstacle at 0 m/s
      if (!nh_private_.getParam ("m_obs_cutoff_0ms", m_obs_cutoff_0ms))
        m_obs_cutoff_0ms = 4000000.0;

      // double, low threshold free space at 1 m/s
      if (!nh_private_.getParam ("m_free_space_cutoff_1ms", m_free_space_cutoff_1ms))
        m_free_space_cutoff_1ms = 2000000.0;

      // double, high threshold obstacle at 1 m/s
      if (!nh_private_.getParam ("m_obs_cutoff_1ms", m_obs_cutoff_1ms))
        m_obs_cutoff_1ms = 4000000.0;

      // double, weight desired direction
      if (!nh_private_.getParam ("m_weight_desired_dir", m_weight_desired_dir))
        m_weight_desired_dir = 10.0;

      // double, weight current direction
      if (!nh_private_.getParam ("m_weight_current_dir", m_weight_current_dir))
        m_weight_current_dir = 1.0;

      // robot radius in mm
      if (!nh_private_.getParam ("m_robot_radius", m_robot_radius))
        m_robot_radius = 300.0;

      if (!nh_private_.getParam ("odom_topic_", odom_topic_))
        odom_topic_ = "odom";

      if (!nh_private_.getParam ("scan_topic_", scan_topic_))
        scan_topic_ = "base_scan/scan";

      reached_goal_ = false;
      ROS_INFO("topic: %s , %s", scan_topic_.c_str(), odom_topic_.c_str());
      nh_private_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      nh_private_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);//default 0.10

      //initializing the object vfh algorithm 
      m_vfh = new VFH_Algorithm(m_cell_size, m_window_diameter, m_sector_angle,
                m_safety_dist_0ms, m_safety_dist_1ms, m_max_speed,
                m_max_speed_narrow_opening, m_max_speed_wide_opening,
                m_max_acceleration, m_min_turnrate, m_max_turnrate_0ms,
                m_max_turnrate_1ms, m_min_turn_radius_safety_factor,
                m_free_space_cutoff_0ms, m_obs_cutoff_0ms, m_free_space_cutoff_1ms,
                m_obs_cutoff_1ms, m_weight_desired_dir, m_weight_current_dir);

      m_vfh->SetRobotRadius(m_robot_radius);
      m_vfh->Init();

      //initialize the copy of the costmap the controller will use
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle gn;
      // subscribe to topics
      scan_subscriber_ = gn.subscribe(scan_topic_, 1, &VFHPlannerROS::scanCallback, this);
      odom_subscriber_ = gn.subscribe(odom_topic_, 1, &VFHPlannerROS::odomCallback, this);
    
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<VFHPlannerConfig>(nh_private_);
      dynamic_reconfigure::Server<VFHPlannerConfig>::CallbackType cb = boost::bind(&VFHPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
//end initialize


  void VFHPlannerROS::odomCallback (const nav_msgs::Odometry::ConstPtr& msg){

    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    m_robotVel = msg->twist.twist.linear.x * 1000.0;

  }

  void VFHPlannerROS::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg){

    unsigned int n = scan_msg->ranges.size();
    for (unsigned i = 0; i < 361; i++)
      m_laser_ranges[i][0] = -1;

    int step=1;
    int startIndex=0;
    float laserSpan = scan_msg->angle_max - scan_msg->angle_min;

    // in case we are using HOKUYO
    if(laserSpan > M_PI || n>180){

      startIndex = (- M_PI/2 - scan_msg->angle_min) /scan_msg->angle_increment;
      float rays_per_degree = (M_PI/180.0)/scan_msg->angle_increment;

      ROS_DEBUG("scanCallback(): startIndex %d, raysxdeg %f", startIndex, rays_per_degree);

      for (unsigned i = 0; i<180; i++){
        step = int(rays_per_degree * i);
        // calculate position in laser frame
        if (startIndex+step > n-1) // probably this is not necessary :/
          step = step-1;

        double r = scan_msg->ranges[startIndex+step]*1000.0;

        if (r<10)
          r = scan_msg->range_max *1000.0;

        ROS_DEBUG("%d:%f\n",i,r);
        m_laser_ranges[i*2][0] = r;
        m_laser_ranges[i*2 + 1][0] = r;
      }
    }else{

      // in case we are using SICK
      for (unsigned i = 0; i<180; i++){

        // calculate position in laser frame
        double r = scan_msg->ranges[i]*1000.0;
        m_laser_ranges[i*2][0] = r;
        m_laser_ranges[i*2 + 1][0] = r;
      }
    }
  }

  bool VFHPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    
    std::vector<geometry_msgs::PoseStamped> local_plan;

    //voglio la posa del robot
    tf::Stamped<tf::Pose> global_pose;
    /**GET ROBOT POSE:
     ** @brief Get the pose of the robot in the global frame of the costmap
      * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
      * @return True if the pose was set successfully, false otherwise
      */
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    //costmap_2d::Costmap2D costmap;
    //costmap_ros_->getCostmapCopy(costmap);
    std::vector<geometry_msgs::PoseStamped> transformed_plan;// is the global_plan but brought in a global frame 
    
    //get the global plan in our frame
    /**transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
        const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
        std::vector<geometry_msgs::PoseStamped>& transformed_plan);
     * @brief  Transforms the global plan of the robot from the planner frame to the local frame
     * @param tf A reference to a transform listener
     * @param global_plan The plan to be transformed
     * @param global_pose The pose of global
     * @param costmap A reference to the costmap being used so the window size for transforming can be computed
     * @param global_frame The frame to transform the plan to
     * @param transformed_plan Populated with the transformed plan
     */
    if(!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, costmap_ros_->getGlobalFrameID(), transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //reduce pose considerate
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);

    //caculate coordinate of goal that it would be the last pose in transformed plan
    if(transformed_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();
    double yaw = tf::getYaw(goal_point.getRotation());
    double goal_th = yaw;

    //ROS_INFO("chosen goal %f, %f", transformed_plan.back().pose.position.x, transformed_plan.back().pose.position.y);
 
    tf::Stamped<tf::Pose> final_goal_point;
    tf::poseStampedMsgToTF(global_plan_.back(), final_goal_point);
    //we assume the global goal is the last point in the global plan
    //just for debug---
    double final_goal_x = final_goal_point.getOrigin().getX();
    double final_goal_y = final_goal_point.getOrigin().getY();
    double final_yaw = tf::getYaw(final_goal_point.getRotation());
    double final_goal_th = final_yaw;
    //---
    double robot_global_x = global_pose.getOrigin().getX();
    double robot_global_y = global_pose.getOrigin().getY();
    double robot_global_th = tf::getYaw(global_pose.getRotation());
    //-----
    
    //ROS_INFO("robot global_pose %f %f %f", robot_global_x, robot_global_y, angles::to_degrees(tf::getYaw(global_pose.getRotation())));
    //ROS_INFO("final goal %f %f %f frame %s", final_goal_x, final_goal_y, angles::to_degrees(final_goal_th), global_plan_.back().header.frame_id.c_str() );
   
    //check to see if we've reached the goal position
    if (xy_tolerance_latch_ || (base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs(angle) <= yaw_goal_tolerance_) {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      }else{
        
        ROS_INFO("goal raggiunto ma orientazione no");
        float temp_goal_th = atan((goal_y - robot_global_y)/(goal_x - robot_global_x));
        if(goal_x < robot_global_x){ 

          temp_goal_th = temp_goal_th+3.14;
        }
    
        float desiredAngle=angles::to_degrees(angles::shortest_angular_distance(tf::getYaw(global_pose.getRotation()),temp_goal_th));    
        float desiredDist= 0;
        float currGoalDistanceTolerance=250;
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        m_vfh->Update_VFH(m_laser_ranges, (int) (m_robotVel), desiredAngle+90,
                          desiredDist, currGoalDistanceTolerance, chosen_speed,
                          chosen_turnrate);
        
        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        odom_helper_.getOdom(base_odom);
                                                                        
        cmd_vel.linear.x=0;
        cmd_vel.linear.y=0;        
        cmd_vel.angular.z= (DEG2RAD(10));

        //ROS_INFO("chosen_speed %d, chosen_turnrate %d  (rad = %f)", chosen_speed,
        //chosen_turnrate, DEG2RAD(chosen_turnrate));
        return true;
      }
      //publish an empty plan because we've reached our goal position
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
      base_local_planner::publishPlan(local_plan, l_plan_pub_);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    float temp_goal_th = atan((goal_y - robot_global_y)/(goal_x - robot_global_x));
    if(goal_x < robot_global_x){ 
        temp_goal_th = temp_goal_th+3.14;
    }

    float desiredAngle=angles::to_degrees(angles::shortest_angular_distance(tf::getYaw(global_pose.getRotation()),temp_goal_th));        
    float desiredDist=sqrt(pow((global_pose.getOrigin().getX()-goal_x),2)+pow((global_pose.getOrigin().getY()-goal_y),2))*1000;//mi serve in mm
    float currGoalDistanceTolerance=250;

    //ROS_INFO("desired dist %f, desired_angle %f", desiredDist/1000, desiredAngle+ 90.0);

    // Choose a new speed and turnrate based on the given laser data and current speed.
    //
    // Units/Senses:
    //  - goal_direction in degrees, 0deg is to the right.
    //  - goal_distance  in mm.
    //  - goal_distance_tolerance in mm.
    //
    m_vfh->Update_VFH(m_laser_ranges, (int) (m_robotVel), desiredAngle+90,
                      desiredDist, currGoalDistanceTolerance, chosen_speed,
                      chosen_turnrate);

    cmd_vel.linear.x=(float)(chosen_speed)/1000.0;
    cmd_vel.angular.z= DEG2RAD(chosen_turnrate);

    //ROS_INFO("chosen_speed %d, chosen_turnrate %d", chosen_speed,
    //        chosen_turnrate);
    ROS_DEBUG("chosen_speed %d, chosen_turnrate %d", chosen_speed,
            chosen_turnrate);
    
    //base_local_planner::publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);

    base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
    base_local_planner::publishPlan(local_plan, l_plan_pub_);
    return true;
  }

  bool VFHPlannerROS::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    return reached_goal_;
  }

  bool VFHPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    ROS_INFO("SET NEW PLAN");
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    reached_goal_ = false;
    return true;
  }
    
};
