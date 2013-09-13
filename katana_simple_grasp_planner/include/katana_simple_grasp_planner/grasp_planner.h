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
 * grasp_planner.h
 *
 *  Created on: 17.11.2012
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef GRASP_PLANNER_H_
#define GRASP_PLANNER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pluginlib/class_loader.h>
#include <kinematics_base/kinematics_base.h>

#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <manipulation_msgs/GraspPlanningAction.h>
#include <manipulation_msgs/GraspPlanningErrorCode.h>

namespace katana_simple_grasp_planner
{

class GraspPlanner
{
public:
  GraspPlanner();
  virtual ~GraspPlanner();

  void main_loop();

private:
  tf::TransformBroadcaster tf_broadcaster_;

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
  boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<manipulation_msgs::GraspPlanningAction> as_;

  sensor_msgs::JointState pre_grasp_joint_state_;
  sensor_msgs::JointState grasp_joint_state_;


  std::vector<tf::Transform> generate_grasps(double x, double y, double z);
  std::vector<double> get_ik(tf::Transform grasp_tf);

  void execute_cb(const manipulation_msgs::GraspPlanningGoalConstPtr &goal);
};

} /* namespace katana_simple_grasp_planner */
#endif /* GRASP_PLANNER_H_ */
