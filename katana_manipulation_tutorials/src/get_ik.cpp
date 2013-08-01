#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_ik");
  ros::NodeHandle rh;

//  ros::service::waitForService("arm_kinematics/get_ik_solver_info");
//  ros::service::waitForService("arm_kinematics/get_ik");
//  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("arm_kinematics/get_ik_solver_info");
//  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>("arm_kinematics/get_ik");

  ros::service::waitForService("kurtana_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("kurtana_arm_kinematics/get_ik");
  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("kurtana_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>("kurtana_arm_kinematics/get_ik");

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  // define the service messages
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "katana_gripper_tool_frame";

  // values for all links set to 0.0 rad (from get_fk):
  gpik_req.ik_request.pose_stamped.header.frame_id = "katana_base_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.3473;
  gpik_req.ik_request.pose_stamped.pose.position.y = -0.0898;
  gpik_req.ik_request.pose_stamped.pose.position.z = 0.3115;
  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.2657;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 0.9641;

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
    else
      ROS_ERROR("Inverse kinematics failed, error code: %d", gpik_res.error_code.val);
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");
  ros::shutdown();
}
