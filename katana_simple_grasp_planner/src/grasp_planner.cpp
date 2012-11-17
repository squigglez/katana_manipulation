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

void GraspPlanner::main_loop() {
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    double roll = 0.0, pitch = 0.0, yaw = M_PI;
    double x = 0.30, y = 0.0, z = 0.0;

    tf::Transform transform;

    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_link", "wrist_link"));

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
