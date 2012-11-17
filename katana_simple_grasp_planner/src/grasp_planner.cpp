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

GraspPlanner::GraspPlanner()
{
  // TODO Auto-generated constructor stub

}

GraspPlanner::~GraspPlanner()
{
  // TODO Auto-generated destructor stub
}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 */
std::vector<tf::Transform> GraspPlanner::generate_grasps(double x, double y, double z)
{
  static const double ANGLE_INC = M_PI / 160;
  static const double SIDE_ANGLE_MIN = -M_PI / 2;
  static const double STRAIGHT_ANGLE_MIN = 0.0;
  static const double ANGLE_MAX = M_PI / 2;

  // how far from the grasp center should the wrist be?
  static const double STANDOFF = -0.20;

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
  for (double roll = 0.0; roll <= M_PI; roll += M_PI)
  {
    double pitch = 0.0;
    for (double yaw = SIDE_ANGLE_MIN + atan2(y, x); yaw <= ANGLE_MAX + atan2(y, x); yaw += ANGLE_INC)
    {
      transform.setOrigin(tf::Vector3(x, y, z));
      transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

      grasps.push_back(transform * standoff_trans);
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

void GraspPlanner::main_loop() {
  ros::Rate loop_rate(10);

  std::vector<tf::Transform> grasps = generate_grasps(0.40, 0.30, 0.0);

  // publish TFs
  for (std::vector<tf::Transform>::iterator it = grasps.begin(); it != grasps.end(); ++it)
  {
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
  ros::init(argc, argv, "katana");
  katana_simple_grasp_planner::GraspPlanner grasp_planner_node;

  grasp_planner_node.main_loop();
  return 0;
}
