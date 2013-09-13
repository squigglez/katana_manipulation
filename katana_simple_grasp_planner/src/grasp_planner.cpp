/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2012 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * grasp_planner.cpp
 *
 *  Created on: 17.11.2012
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <katana_simple_grasp_planner/grasp_planner.h>
#include <math.h>

namespace katana_simple_grasp_planner
{

GraspPlanner::GraspPlanner() :
    kinematics_loader_("kinematics_base", "kinematics::KinematicsBase"),
    as_(nh_, "/plan_point_cluster_grasp", boost::bind(&GraspPlanner::execute_cb, this, _1), false)
{
  as_.start();

  // this is usually configured via pr2_gripper_grasp_planner_cluster/config
  pre_grasp_joint_state_.name.push_back("katana_l_finger_joint");
  pre_grasp_joint_state_.name.push_back("katana_r_finger_joint");
  pre_grasp_joint_state_.position.push_back(0.30);
  pre_grasp_joint_state_.position.push_back(0.30);
  pre_grasp_joint_state_.effort.push_back(100.0);
  pre_grasp_joint_state_.effort.push_back(100.0);

  grasp_joint_state_.name = pre_grasp_joint_state_.name;
  grasp_joint_state_.position.push_back(-0.44);
  grasp_joint_state_.position.push_back(-0.44);
  grasp_joint_state_.effort.push_back(50.0);
  grasp_joint_state_.effort.push_back(50.0);

  // see arm_kinematics_constraint_aware/src/arm_kinematics_solver_constraint_aware.cpp
  try
  {
      kinematics_solver_ = kinematics_loader_.createInstance("arm_kinematics_constraint_aware/KDLArmKinematicsPlugin");
  }
  catch(pluginlib::PluginlibException& ex)
  {
      ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
      return;
  }

  std::string group_name = "arm";
  std::string base_name = "katana_base_link";
  std::string tip_name = "katana_motor5_wrist_roll_link";

  if(kinematics_solver_->initialize(group_name,
                                    base_name,
                                    tip_name,
                                    .025)) {
  } else {
    ROS_ERROR_STREAM("Initialize is failing for " << group_name);
    return;
  }

  ROS_INFO("katana_simple_grasp_planner initialized.");
}

GraspPlanner::~GraspPlanner()
{
}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 */
std::vector<tf::Transform> GraspPlanner::generate_grasps(double x, double y, double z)
{
  static const double ANGLE_INC = M_PI / 16;
  static const double STRAIGHT_ANGLE_MIN = 0.0 + ANGLE_INC;  // + ANGLE_INC, because 0 is already covered by side grasps
  static const double ANGLE_MAX = M_PI / 2;

  // how far from the grasp center should the wrist be?
  static const double STANDOFF = -0.12;

  std::vector<tf::Transform> grasps;

  tf::Transform transform;

  tf::Transform standoff_trans;
  standoff_trans.setOrigin(tf::Vector3(STANDOFF, 0.0, 0.0));
  standoff_trans.setRotation(tf::createIdentityQuaternion());


  // ----- side grasps
  //
  //  1. side grasp (xy-planes of `katana_motor5_wrist_roll_link` and of `katana_base_link` are parallel):
  //     - standard: `rpy = (0, 0, *)` (orientation of `katana_motor5_wrist_roll_link` in `katana_base_link` frame)
  //     - overhead: `rpy = (pi, 0, *)`
  transform.setOrigin(tf::Vector3(x, y, z));

  for (double roll = 0.0; roll <= M_PI; roll += M_PI)
  {
    double pitch = 0.0;

    // add yaw = 0 first, then +ANGLE_INC, -ANGLE_INC, 2*ANGLE_INC, ...
    // reason: grasps with yaw near 0 mean that the approach is from the
    // direction of the arm; it is usually easier to place the object back like
    // this
    for (double yaw = ANGLE_INC; yaw <= ANGLE_MAX; yaw += ANGLE_INC)
    {
      // + atan2 to center the grasps around the vector from arm to object
      transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw + atan2(y, x)));
      grasps.push_back(transform * standoff_trans);

      if (yaw != 0.0)
      {
        transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, -yaw + atan2(y, x)));
        grasps.push_back(transform * standoff_trans);
      }
    }
  }

  // ----- straight grasps
  //
  //  2. straight grasp (xz-plane of `katana_motor5_wrist_roll_link` contains z axis of `katana_base_link`)
  //     - standard: `rpy = (0, *, atan2(y_w, x_w))`   (x_w, y_w = position of `katana_motor5_wrist_roll_link` in `katana_base_link` frame)
  //     - overhead: `rpy = (pi, *, atan2(y_w, x_w))`
  for (double roll = 0.0; roll <= M_PI; roll += M_PI)
  {
    for (double pitch = STRAIGHT_ANGLE_MIN; pitch <= ANGLE_MAX; pitch += ANGLE_INC)
    {
      double yaw = atan2(y, x);
      transform.setOrigin(tf::Vector3(x, y, z));
      transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

      grasps.push_back(transform * standoff_trans);
    }
  }

  return grasps;
}

std::vector<double> GraspPlanner::get_ik(tf::Transform grasp_tf)
{
  geometry_msgs::Pose ik_pose;
  tf::poseTFToMsg(grasp_tf, ik_pose);

  std::vector<double> ik_seed_state;
  ik_seed_state.resize(5, 0.0);

  std::vector<double> solution;
  int error_code;   // see arm_navigation_msgs/msg/ArmNavigationErrorCodes.msg

  kinematics_solver_->getPositionIK(ik_pose, ik_seed_state, solution, error_code);

  if (error_code == 1)
    ROS_DEBUG("IK solution: %f %f %f %f %f", solution[0], solution[1], solution[2], solution[3], solution[4]);
  else
    ROS_INFO("no IK found (error %d)", error_code);

  return solution;
}

void GraspPlanner::execute_cb(const manipulation_msgs::GraspPlanningGoalConstPtr &goal)
{
  manipulation_msgs::GraspPlanningFeedback feedback;
  manipulation_msgs::GraspPlanningResult result;


  // ----- compute the center point of the object
  // just calculate the average
  sensor_msgs::PointCloud cloud = goal->target.cluster;
  float x = 0.0, y = 0.0, z = 0.0;
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    x += cloud.points[i].x;
    y += cloud.points[i].y;
    z += cloud.points[i].z;
  }
  x /= cloud.points.size();
  y /= cloud.points.size();
  z /= cloud.points.size();

  std::vector<tf::Transform> grasp_tfs = generate_grasps(x, y, z);

  for (std::vector<tf::Transform>::iterator it = grasp_tfs.begin(); it != grasp_tfs.end(); ++it)
  {
    // skip grasps without IK solution
    if (get_ik(*it).size() == 0)
      continue;

    manipulation_msgs::Grasp grasp;

    // TODO:
    //# A name for this grasp
    //string id

    //# The internal posture of the hand for the pre-grasp
    //# only positions are used
    //sensor_msgs/JointState pre_grasp_posture
    grasp.pre_grasp_posture = pre_grasp_joint_state_;

    //# The internal posture of the hand for the grasp
    //# positions and efforts are used
    //sensor_msgs/JointState grasp_posture
    grasp.grasp_posture = grasp_joint_state_;

    //# The position of the end-effector for the grasp relative to a reference frame
    //# (that is always specified elsewhere, not in this message)
    //geometry_msgs/PoseStamped grasp_pose
    tf::poseTFToMsg(*it, grasp.grasp_pose.pose);
    // TODO: grasp_pose.header

    //# The estimated probability of success for this grasp, or some other
    //# measure of how "good" it is.
    //float64 grasp_quality
    grasp.grasp_quality = 0.5;

    //# The approach motion
    //GripperTranslation approach
    //# the direction of the translation
    // TODO: geometry_msgs/Vector3Stamped direction
    //# the desired translation distance
    grasp.approach.desired_distance = 0.10;
    //# the min distance that must be considered feasible before the
    //# grasp is even attempted
    grasp.approach.min_distance = 0.05;

    // TODO:
    //# The retreat motion
    //GripperTranslation retreat

    //# the maximum contact force to use while grasping (<=0 to disable)
    //float32 max_contact_force
    grasp.max_contact_force = -1.0;

    // TODO:
    //# an optional list of obstacles that we have semantic information about
    //# and that can be touched/pushed/moved in the course of grasping
    //string[] allowed_touch_objects

    //tf_broadcaster_.sendTransform(tf::StampedTransform(*it, ros::Time::now(), "katana_base_link", "wrist_link"));
    // ros::Duration(2.0).sleep();

    feedback.grasps.push_back(grasp);

    // don't publish feedback for now
    // as_->publishFeedback(feedback);
  }

  result.grasps = feedback.grasps;
  if (result.grasps.size() == 0)
    result.error_code.value = result.error_code.OTHER_ERROR;
  else
    result.error_code.value = result.error_code.SUCCESS;

  ROS_INFO("Returning %zu grasps.", result.grasps.size());

  as_.setSucceeded(result);
}



void GraspPlanner::main_loop() {
  ros::Rate loop_rate(0.5);

  std::vector<tf::Transform> grasps = generate_grasps(0.30, 0.20, 0.0);

  // publish TFs
  for (std::vector<tf::Transform>::iterator it = grasps.begin(); it != grasps.end(); ++it)
  {
    std::vector<double> ik_sol = get_ik(*it);

    if (ik_sol.size() > 0)
      tf_broadcaster_.sendTransform(tf::StampedTransform(*it, ros::Time::now(), "katana_base_link", "wrist_link"));

    if (!ros::ok())
      break;

    ros::spinOnce();
    loop_rate.sleep();
  }
}

} /* namespace katana_simple_grasp_planner */


int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_simple_grasp_planner");
  katana_simple_grasp_planner::GraspPlanner grasp_planner_node;

  //grasp_planner_node.main_loop();
  ros::spin();
  return 0;
}
