#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <std_srvs/Empty.h>

static const double TABLE_HEIGHT = 0.28;

tf::TransformListener *tf_listener;


static const size_t NUM_JOINTS = 5;

bool move_to_joint_goal(std::vector<arm_navigation_msgs::JointConstraint> joint_constraints,
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> &move_arm) {

  arm_navigation_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");

  goal.motion_plan_request.goal_constraints.joint_constraints = joint_constraints;


  bool finished_within_time = false;
  move_arm.sendGoal(goal);
  finished_within_time = move_arm.waitForResult(ros::Duration(40.0));
  if (!finished_within_time)
  {
    move_arm.cancelGoal();
    ROS_INFO("Timed out achieving goal!");
    return false;
  }
  else
  {
    actionlib::SimpleClientGoalState state = move_arm.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
      ROS_INFO("Action finished: %s",state.toString().c_str());
    else
      ROS_INFO("Action failed: %s",state.toString().c_str());

    return success;
  }
}


/**
 * return nearest object to point
 */
bool nearest_object(std::vector<object_manipulation_msgs::GraspableObject>& objects, geometry_msgs::PointStamped& reference_point, int& object_ind)
{
  geometry_msgs::PointStamped point;

  // convert point to base_link frame
  tf_listener->transformPoint("/base_link", reference_point, point);

  // find the closest object
  double nearest_dist = 1e6;
  int nearest_object_ind = -1;

  for (size_t i = 0; i < objects.size(); ++i)
  {
    sensor_msgs::PointCloud cloud;
    tf_listener->transformPointCloud("/base_link", objects[i].cluster, cloud);

    // calculate average
    float x = 0.0, y = 0.0, z = 0.0;
    for (size_t j = 0; j < cloud.points.size(); ++j)
    {
      x += cloud.points[j].x;
      y += cloud.points[j].y;
      z += cloud.points[j].z;
    }
    x /= cloud.points.size();
    y /= cloud.points.size();
    z /= cloud.points.size();

    double dist = sqrt(pow(x - point.point.x, 2.0) + pow(y - point.point.y, 2.0) + pow(z - point.point.z, 2.0));
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      nearest_object_ind = i;
    }
  }

  if (nearest_object_ind > -1)
  {
    ROS_INFO("nearest object ind: %d (distance: %f)", nearest_object_ind, nearest_dist);
    object_ind = nearest_object_ind;
    return true;
  } else
  {
    ROS_ERROR("No nearby objects. Unable to select grasp target");
    return false;
  }
}

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "pick_and_place_app");
  ros::NodeHandle nh;

  //set service and action names
  const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
  const std::string COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing";
  const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
  const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
  const std::string MOVE_ARM_ACTION_NAME = "/move_arm";
  const std::string COLLIDER_RESET_SERVICE_NAME = "/collider_node/reset";

  // create TF listener
  tf_listener = new tf::TransformListener();

  //create service and action clients
  ros::ServiceClient object_detection_srv;
  ros::ServiceClient collision_processing_srv;
  ros::ServiceClient collider_reset_srv;
  actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction>
    pickup_client(PICKUP_ACTION_NAME, true);
  actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction>
    place_client(PLACE_ACTION_NAME, true);
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(MOVE_ARM_ACTION_NAME, true);

  //wait for detection client
  while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME,
                                        ros::Duration(2.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for object detection service to come up");
  }
  if (!nh.ok()) exit(0);
  object_detection_srv =
    nh.serviceClient<tabletop_object_detector::TabletopDetection>
    (OBJECT_DETECTION_SERVICE_NAME, true);

  //wait for collision map processing client
  while (!ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for collision processing service to come up");
  }
  if (!nh.ok())
    exit(0);
  collision_processing_srv = nh.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>(
      COLLISION_PROCESSING_SERVICE_NAME, true);

  //wait for collider reset service
  while (!ros::service::waitForService(COLLIDER_RESET_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for collider reset service to come up");
  }
  if (!nh.ok())
    exit(0);
  collider_reset_srv = nh.serviceClient<std_srvs::Empty>(
      COLLIDER_RESET_SERVICE_NAME, true);

  //wait for pickup client
  while(!pickup_client.waitForServer(ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
  }
  if (!nh.ok()) exit(0);

  //wait for place client
  while(!place_client.waitForServer(ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
  }
  if (!nh.ok()) exit(0);

  //wait for move_arm action client
  while(!move_arm.waitForServer(ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO_STREAM("Waiting for action client " << MOVE_ARM_ACTION_NAME);
  }
  if (!nh.ok()) exit(0);





  // ----- reset collision map
  ROS_INFO("Clearing collision map");
  std_srvs::Empty empty;
  if (!collider_reset_srv.call(empty))
  {
    ROS_ERROR("Collider reset service failed");
    return -1;
  }
  ros::Duration(5.0).sleep();   // wait for collision map to be completely cleared

  // ----- move arm out of the way
  std::vector<std::string> names(NUM_JOINTS);
  names[0] = "katana_motor1_pan_joint";
  names[1] = "katana_motor2_lift_joint";
  names[2] = "katana_motor3_lift_joint";
  names[3] = "katana_motor4_lift_joint";
  names[4] = "katana_motor5_wrist_roll_joint";


  std::vector<arm_navigation_msgs::JointConstraint> joint_constraints(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_constraints[i].joint_name = names[i];
    joint_constraints[i].tolerance_below = 0.1;
    joint_constraints[i].tolerance_above = 0.1;
  }

  //  - post_calibration_posture:
  //      name: ['katana_motor1_pan_joint', 'katana_motor2_lift_joint', 'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint']
  //      position: [-2.9641690268167444, 2.13549384276445, -2.1556486321117725, -1.971949347057968, -2.9318804356548496]
  //  - arm_away_posture:
  //      name: ['katana_motor1_pan_joint', 'katana_motor2_lift_joint', 'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint']
  //      position: [0.0, 2.13549384276445, -2.1556486321117725, -1.971949347057968, 0.0]

  joint_constraints[0].position =  0.0;
  joint_constraints[1].position =  2.13549384276445;
  joint_constraints[2].position = -2.1556486321117725;
  joint_constraints[3].position = -1.971949347057968;
  joint_constraints[4].position = 0.0;

  ROS_INFO("Moving arm away");
  bool success;
  success = move_to_joint_goal(joint_constraints, move_arm);
  if (!success)
    return -1;

  ros::Duration(2.0).sleep();   // only necessary for Gazebo (the simulated Kinect point cloud lags, so we need to wait for it to settle)

  // ----- reset collision map
  ROS_INFO("Clearing collision map");
  if (!collider_reset_srv.call(empty))
  {
    ROS_ERROR("Collider reset service failed");
    return -1;
  }
  ros::Duration(5.0).sleep();   // wait for collision map to be completely cleared

  // ----- call the tabletop detection
  ROS_INFO("Calling tabletop detector");
  tabletop_object_detector::TabletopDetection detection_call;
  //we want recognized database objects returned
  //set this to false if you are using the pipeline without the database
  detection_call.request.return_clusters = true;
  //we want the individual object point clouds returned as well
  detection_call.request.return_models = false;
  if (!object_detection_srv.call(detection_call))
  {
    ROS_ERROR("Tabletop detection service failed");
    return -1;
  }
  if (detection_call.response.detection.result !=
      detection_call.response.detection.SUCCESS)
  {
    ROS_ERROR("Tabletop detection returned error code %d",
              detection_call.response.detection.result);
    return -1;
  }
  if (detection_call.response.detection.clusters.empty() &&
      detection_call.response.detection.models.empty() )
  {
    ROS_ERROR("The tabletop detector detected the table, but found no objects");
    return -1;
  }


  // ----- call collision map processing
  ROS_INFO("Calling collision map processing");
  tabletop_collision_map_processing::TabletopCollisionMapProcessing
    processing_call;
  //pass the result of the tabletop detection
  processing_call.request.detection_result = detection_call.response.detection;
  //ask for the exising map and collision models to be reset
  //processing_call.request.reset_static_map = true;
  processing_call.request.reset_collision_models = true;
  processing_call.request.reset_attached_models = true;
  //ask for a new static collision map to be taken with the laser
  //after the new models are added to the environment
  //processing_call.request.take_static_collision_map = true;
  //ask for the results to be returned in base link frame
  processing_call.request.desired_frame = "katana_base_link";
  if (!collision_processing_srv.call(processing_call))
  {
    ROS_ERROR("Collision map processing service failed");
    return -1;
  }
  //the collision map processor returns instances of graspable objects
  if (processing_call.response.graspable_objects.empty())
  {
    ROS_ERROR("Collision map processing returned no graspable objects");
    return -1;
  }


  // ----- pick up object near point: 25 cm in front and 15 cm to the right of the robot
  geometry_msgs::PointStamped pickup_point;
  pickup_point.header.frame_id = "/base_footprint";
  pickup_point.point.x = 0.25;
  pickup_point.point.y = -0.15;
  pickup_point.point.z = TABLE_HEIGHT;

  //call object pickup
  ROS_INFO("Calling the pickup action");
  object_manipulation_msgs::PickupGoal pickup_goal;
  //pass one of the graspable objects returned by the collission map processor
  int object_to_pick_ind;

  if (!nearest_object(processing_call.response.graspable_objects, pickup_point, object_to_pick_ind))
    return -1;
  pickup_goal.target = processing_call.response.graspable_objects[object_to_pick_ind];

  //pass the name that the object has in the collision environment
  //this name was also returned by the collision map processor
  pickup_goal.collision_object_name =
    processing_call.response.collision_object_names.at(object_to_pick_ind);
  //pass the collision name of the table, also returned by the collision
  //map processor
  pickup_goal.collision_support_surface_name =
    processing_call.response.collision_support_surface_name;
  //pick up the object with the right arm
  pickup_goal.arm_name = "arm";
  pickup_goal.allow_gripper_support_collision = true;
  //pickup_goal.desired_approach_distance = 0.06;
  //pickup_goal.min_approach_distance = 0.04;

  //we will be lifting the object along the "vertical" direction
  //which is along the z axis in the base_link frame
  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = "katana_base_link";
  direction.vector.x = 0;
  direction.vector.y = 0;
  direction.vector.z = 1;
  pickup_goal.lift.direction = direction;
  //request a vertical lift of 10cm after grasping the object
  pickup_goal.lift.desired_distance = 0.1;
  pickup_goal.lift.min_distance = 0.05;
  //do not use tactile-based grasping or tactile-based lift
  pickup_goal.use_reactive_lift = false;
  pickup_goal.use_reactive_execution = false;
  //send the goal
  pickup_client.sendGoal(pickup_goal);
  while (!pickup_client.waitForResult(ros::Duration(10.0)))
  {
    ROS_INFO("Waiting for the pickup action...");
    if (!nh.ok())
      return -1;
  }
  object_manipulation_msgs::PickupResult pickup_result = *(pickup_client.getResult());
  if (pickup_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Pickup succeeded.");
  } else
  {
    ROS_ERROR("The pickup action has failed with result code %d",
              pickup_result.manipulation_result.value);
    return -1;
  }

  //remember where we picked the object up from
  geometry_msgs::PoseStamped pickup_location;
  //for unrecognized point clouds, the location of the object is considered
  //to be the origin of the frame that the cluster is in
  pickup_location.header =
      processing_call.response.graspable_objects.at(object_to_pick_ind).cluster.header;
    //identity pose
    pickup_location.pose.orientation.w = 1;

  //create a place location, offset by 30 cm from the pickup location
  geometry_msgs::PoseStamped place_location = pickup_location;
  place_location.header.stamp = ros::Time::now();
  place_location.pose.position.y += 0.3;

  // ----- put the object down
  ROS_INFO("Calling the place action");
  object_manipulation_msgs::PlaceGoal place_goal;
  //place at the prepared location
  place_goal.place_locations.push_back(place_location);
  //the collision names of both the objects and the table
  //same as in the pickup action
  place_goal.collision_object_name =
    processing_call.response.collision_object_names.at(object_to_pick_ind);
  place_goal.collision_support_surface_name =
    processing_call.response.collision_support_surface_name;
  //information about which grasp was executed on the object, returned by
  //the pickup action
  place_goal.grasp = pickup_result.grasp;
  //use the arm to place
  place_goal.arm_name = "arm";
  //padding used when determining if the requested place location
  //would bring the object in collision with the environment
  place_goal.place_padding = 0.02;
  //how much the gripper should retreat after placing the object
  place_goal.desired_retreat_distance = 0.1;
  place_goal.min_retreat_distance = 0.05;
  //we will be putting down the object along the "vertical" direction
  //which is along the z axis in the base_link frame
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = "katana_base_link";
  direction.vector.x = 0;
  direction.vector.y = 0;
  direction.vector.z = -1;
  place_goal.approach.direction = direction;
  //request a vertical put down motion of 10cm before placing the object
  place_goal.approach.desired_distance = 0.1;
  place_goal.approach.min_distance = 0.05;
  //we are not using tactile based placing
  place_goal.use_reactive_place = false;
  //send the goal
  place_client.sendGoal(place_goal);
  while (!place_client.waitForResult(ros::Duration(10.0)))
  {
    ROS_INFO("Waiting for the place action...");
    if (!nh.ok())
      return -1;
  }
  object_manipulation_msgs::PlaceResult place_result =
    *(place_client.getResult());
  if (place_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR("Place failed with error code %d",
              place_result.manipulation_result.value);
    return -1;
  }

  // ----- move arm away again
  ROS_INFO("Moving arm away");
  success = move_to_joint_goal(joint_constraints, move_arm);
  if (!success)
    return -1;

  //success!
  ROS_INFO("Success! Object moved.");
  return 0;
}
