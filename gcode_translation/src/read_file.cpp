#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>

#include <sstream>
#include <fstream>

#include <time.h>
#include <math.h>

ros::WallTime start_, end_;
int main(int argc, char **argv)
{
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "gcode_translation");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //----------------------------
  //Setup
  //----------------------------
  static const std::string PLANNING_GROUP = "arm";
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  std::string chain_start, chain_end, urdf_param;
  double timeout;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.005);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
 
  double eps = 1e-5;
  node_handle.param("eps", eps, 1e-5);
  ROS_INFO_STREAM("eps :" << eps);
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Distance);
  TRAC_IK::TRAC_IK tracik_solver2(chain_start, chain_end, urdf_param, timeout*10, eps, TRAC_IK::Distance);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  ROS_INFO_NAMED("moveo", "TRAC-IK setup");
  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return -1;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return -1;
  }
  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  ROS_INFO("Using %d joints", chain.getNrOfJoints());
  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  //-----------------------------
  //Getting Basic Information
  //-----------------------------

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("moveo", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  KDL::JntArray result;
  KDL::Vector end_effector_target_vol;
  KDL::Rotation end_effector_target_rot;
 
  int rc;
  std::vector<KDL::JntArray> JointList;
  geometry_msgs::Pose target_pose1;
  std::vector<double> target_joints;
  robot_state::RobotState start_state(*move_group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =
                start_state.getJointModelGroup(move_group.getName());
  KDL::Vector target_bounds_rot(0, 0, 2* M_PI), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);
  
  std::string gcode_in, gcode_out;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out")); 
  std::ifstream input_file(gcode_in);
  if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);
  
  std::ofstream output_file(gcode_out);
  if(!output_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_out);
  
  std::string line;
  std::string file_line;
  bool check = 0;
  int second_execution = 0;
  while(ros::ok()){
    while(getline(input_file, line)){
      std::istringstream get_line(line);
      file_line = get_line.str();
      if(file_line.find('G') < 1){
        if(file_line.find('0') < 2){
          check = 1;
        }
      }
      if(file_line.find(';') < 1){
        if(file_line.find('G') < 9){
          if(file_line.find('c') < 10){
            output_file << file_line << std::endl;
            output_file.close();
            end_ = ros::WallTime::now();
            double execution_time = (end_ - start_).toNSec() * 1e-9;
            ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
            ROS_INFO_STREAM("second_execution: " << second_execution);
            ros::shutdown();
            return 0;
          }
        }
        output_file << file_line << std::endl;
      }
      else if(check == 1){
        if(file_line.find('G') < 1){
          if(file_line.find('0') < 2 || file_line.find('1') < 2){
            output_file << file_line[0] << file_line[1];
            size_t colon_pos_X = file_line.find('X');
            if(colon_pos_X < 100){
              end_effector_target_vol.data[0] = stod(file_line.substr(colon_pos_X+1))*1e-3;
            }
            size_t colon_pos_Y = file_line.find('Y');
            if(colon_pos_Y < 100){
              end_effector_target_vol.data[1] = (stod(file_line.substr(colon_pos_Y+1))*1e-3)+0.35;
            }
            size_t colon_pos_Z = file_line.find('Z');
            if(colon_pos_Z < 100){
              end_effector_target_vol.data[2] = stod(file_line.substr(colon_pos_Z+1))*1e-3;
            }
            KDL::Frame end_effector_pose(end_effector_target_rot, end_effector_target_vol);
            rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result, target_bounds);
            if(rc < 0){
              rc = tracik_solver2.CartToJnt(nominal, end_effector_pose, result, target_bounds);
              second_execution++;
            }
            ROS_INFO_STREAM("rc :" << rc);
            target_pose1.position.x    = end_effector_pose.p.x();
            target_pose1.position.y    = end_effector_pose.p.y();
            target_pose1.position.z    = end_effector_pose.p.z();
            end_effector_pose.M.GetQuaternion	(target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z, target_pose1.orientation.w);
            if(rc >= 0){
              for(int i = 0; i < chain.getNrOfJoints(); i++){
                switch (i){
                  case 0:
                  output_file << " J" << int(result.data(i)*65600/(2*M_PI));
                  break;
                  case 1:
                  output_file << " A" << int(result.data(i)*18000/(2*M_PI));
                  break;
                  case 2:
                  output_file << " B" << int(result.data(i)*144000/(2*M_PI));
                  break;
                  case 3:
                  output_file << " C" << int(result.data(i)*6560/(2*M_PI));
                  break;
                  case 4:
                  output_file << " D" << int(result.data(i)*28800/(2*M_PI));
                  break;
                }
              }
              for(int j = 2;j < file_line.length(); j++){
                output_file << file_line[j];
              }
              output_file << std::endl;
            }
            else{
              ros::shutdown();
              return 0;
            }
          }
        }
        else{
          output_file << file_line << std::endl;
        }
      }
      else{
        output_file << file_line << std::endl;
      }
    }
  }
}
