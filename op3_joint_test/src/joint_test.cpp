#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

#include <fstream>
//#include <time.h>

void imuHandlerCallback(const sensor_msgs::Imu& msg);
void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

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
  ros::init(argc, argv, "joint_test");

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  sync_write_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
  write_joint_pub2 = nh.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);

  imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, imuHandlerCallback);
  read_joint_sub = nh.subscribe("/robotis/present_joint_states", 1, jointstatesCallback);

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
  if(control_module == Framework)
    write_joint_pub.publish(write_msg);
  else if(control_module == DirectControlModule)
    write_joint_pub2.publish(write_msg);
  
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
  ROS_INFO("Start Joint Test");

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
