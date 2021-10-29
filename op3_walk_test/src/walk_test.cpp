#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/JointCtrlModule.h"

// walking demo
#include <yaml-cpp/yaml.h>
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

// Preview walking
//#include "op3_online_walking_module_msgs/FootStepCommand.h"
//#include "op3_online_walking_module_msgs/WalkingParam.h"
#include "op3_online_walking_module_msgs/JointPose.h"
//#include "op3_online_walking_module_msgs/Step2DArray.h"
//#include "humanoid_nav_msgs/PlanFootsteps.h"

#include <fstream>
//#include <time.h>

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)

void imuHandlerCallback(const sensor_msgs::Imu& msg);
void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();
void setModuleToDemo();
void readyToWalk();
void sendJointPoseMsg(op3_online_walking_module_msgs::JointPose msg);
void setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg);

void parseJointNameFromYaml(const std::string &path);
bool getJointNameFromID(const int &id, std::string &joint_name);

// Walking
void setWalkingCommand(const std::string &command);

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher sync_write_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Publisher write_joint_pub2;
ros::Subscriber imu_sub;
ros::Subscriber read_joint_sub;
ros::Publisher module_control_pub_;

std::map<int, std::string> id_joint_table_;

// Walking
ros::Publisher set_walking_command_pub;
ros::Publisher set_walking_param_pub;
ros::ServiceClient get_walking_param_client_;
ros::Publisher joint_pose_msg_pub_;

ros::ServiceClient set_joint_module_client;

int control_module = None;
bool demo_ready = false;

int counter = 0;
sensor_msgs::Imu imu;

ros::Time ros_begin;
std::ofstream log_file;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "walk_test");

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  sync_write_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
  write_joint_pub2 = nh.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);
  module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);

  imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, imuHandlerCallback);
  read_joint_sub = nh.subscribe("/robotis/present_joint_states", 1, jointstatesCallback);
  
  //init_gyro_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  //set_head_joint_angle_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);
  //
  //current_joint_states_sub_ = nh.subscribe("/robotis/present_joint_states", 10, &QNodeOP3::updateHeadJointStatesCallback, this);
  //
  // Walking
  set_walking_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  get_walking_param_client_ = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");
  joint_pose_msg_pub_ = nh.advertise<op3_online_walking_module_msgs::JointPose>("/robotis/online_walking/goal_joint_pose", 0);

  // service
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of op3_manager
  std::string manager_name = "/op3_manager";
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true)
    {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  char date[64];
  time_t t = ros::WallTime::now().sec;
  int r = strftime(date, sizeof(date), "%Y%m%d%H%M%S", localtime(&t));
  std::string log_file_name = "/tmp/RobotisOP3-" + std::string(date) + ".csv";
  log_file.open(log_file_name, std::ios::out);
  ros_begin = ros::Time::now();

  control_module = Framework;
  readyToDemo();

  // Walking
  parseJointNameFromYaml("/home/grxuser/catkin_ws/src/ROBOTIS-OP3-Komix/op3_walk_test/config/gui_config.yaml");
  setModuleToDemo();
  ROS_INFO("Set module to walking");
  ROS_INFO("Go to walk pose");
  readyToWalk();
  ros::Duration(4.0).sleep();
  ROS_INFO("[start] Walking");
  setWalkingCommand("start");

  //node loop
  while (ros::ok())
  {
    // process

    //execute pending callbacks
    ros::spinOnce();
    
    //relax to fit output rate
    loop_rate.sleep();
  }

  log_file.close();

  //exit program
  return 0;
}

// Walking
void setWalkingCommand(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  set_walking_command_pub.publish(_commnd_msg);
}

void imuHandlerCallback(const sensor_msgs::Imu& msg)
{
  imu = msg;
  /*
  // starting demo using robotis_controller
  if (msg->data == "mode")
  {
    control_module = Framework;
    ROS_INFO("Button : mode | Framework");
    readyToDemo();
  }
  // starting demo using direct_control_module
  else if (msg->data == "start")
  {
    control_module = DirectControlModule;
    ROS_INFO("Button : start | Direct control module");
    readyToDemo();
  }
  // torque on all joints of ROBOTIS-OP3
  else if (msg->data == "user")
  {
    torqueOnAll();
    control_module = None;
  }
  */
}

void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(control_module == None)
    return;

  sensor_msgs::JointState write_msg;
  write_msg.header = msg->header;

  if (counter >= 120) counter = 0;
  double joint_position = M_PI/180.0*30*sin(2*M_PI*counter/120.0);
  counter++;
  write_msg.name.push_back("head_pan");
  write_msg.position.push_back(joint_position);

  // publish a message to set the joint angles
  /*
  if(control_module == Framework)
    write_joint_pub.publish(write_msg);
  else if(control_module == DirectControlModule)
    write_joint_pub2.publish(write_msg);
  */
  
  // log
  ros::Duration ros_duration = ros::Time::now() - ros_begin;
  log_file << std::fixed << std::setprecision(3) << ros_duration.toSec() << ", ";
  log_file << std::defaultfloat;
  
  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    
    std::string joint_name = msg->name[ix];
    double joint_position = msg->position[ix];
    if(joint_name == "head_pan")
    {
      log_file << joint_position << ", ";
    }
    
    // log_file << msg->position[ix] << ", ";
  }

  log_file << imu.orientation.x << ", "
	   << imu.orientation.y << ", "
	   << imu.orientation.z << ", "
	   << imu.orientation.w << ", "
	   << imu.angular_velocity.x << ", "
	   << imu.angular_velocity.y << ", "
	   << imu.angular_velocity.z << ", "
	   << imu.linear_acceleration.x << ", "
	   << imu.linear_acceleration.y << ", "
	   << imu.linear_acceleration.z << std::endl;
}

void readyToDemo()
{
  ROS_INFO("Start Walk Test");

  torqueOnAll();
  ROS_INFO("Torque on All joints");

  // send message for going init posture
  goInitPose();
  ROS_INFO("Go Init pose");

  // wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  // change the module for demo
  if(control_module == Framework)
  {
    setModule("none");
    ROS_INFO("Change module to none");
  }
  else if(control_module == DirectControlModule)
  {
    setModule("direct_control_module");
    ROS_INFO("Change module to direct_control_module");
  }
  else
    return;
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub.publish(init_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }

  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return ;
}

void torqueOnAll()
{
  std_msgs::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub.publish(check_msg);
}

void parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = id_sub_node.begin(); _it != id_sub_node.end(); ++_it)
  {
    int joint_id;
    std::string joint_name;

    joint_id = _it->first.as<int>();
    joint_name = _it->second.as<std::string>();

    id_joint_table_[joint_id] = joint_name;
    //joint_id_table_[joint_name] = joint_id;

    //if (debug_)
    //  std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }

  /*
  // parse module
  std::vector<std::string> modules = doc["module_list"].as<std::vector<std::string> >();

  int module_index = 0;
  for (std::vector<std::string>::iterator modules_it = modules.begin(); modules_it != modules.end(); ++modules_it)
  {
    std::string module_name = *modules_it;

    index_mode_table_[module_index] = module_name;
    mode_index_table_[module_name] = module_index++;

    using_mode_table_[module_name] = false;
  }

  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int key_index;
    std::string module_name;

    key_index = yaml_it->first.as<int>();
    module_name = yaml_it->second.as<std::string>();

    module_table_[key_index] = module_name;
    if (debug_)
      std::cout << "Preset : " << module_name << std::endl;
  }
  */
}

// joint id -> joint name
bool getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;

  map_it = id_joint_table_.find(id);
  if (map_it == id_joint_table_.end())
    return false;

  joint_name = map_it->second;
  return true;
}

void setModuleToDemo()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string body_module = "walking_module";
  std::string head_module = "head_control_module";

  for (int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    if (ix <= 18)
      control_msg.module_name.push_back(body_module);
    else
      control_msg.module_name.push_back(head_module);

  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  setJointControlMode(control_msg);
}

void readyToWalk()
{
  //for walking
  std::string ini_pose_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/init_pose.yaml";
  ROS_INFO(ROS_PACKAGE_NAME);
  //ROS_INFO(ini_pose_path.c_str());
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(ini_pose_path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load yaml file. [" << ini_pose_path << "]");
    return;
  }
  
  op3_online_walking_module_msgs::JointPose msg;
  
  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;
  
  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();

    msg.pose.name.push_back(joint_name);
    msg.pose.position.push_back(value * DEG2RAD);
  }

  sendJointPoseMsg( msg );
}

void sendJointPoseMsg(op3_online_walking_module_msgs::JointPose msg)
{
  joint_pose_msg_pub_.publish( msg );

  //log( Info , "Send Joint Pose Msg" );
}

// set mode(module) to each joint
void setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg)
{
  module_control_pub_.publish(msg);
}
