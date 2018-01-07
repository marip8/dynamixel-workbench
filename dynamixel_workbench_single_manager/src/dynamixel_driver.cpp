/*******************************************************************************
* Copyright (c) 2017, Southwest Research Institute, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of Southwest Research Institute nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/*
 * dynamixel_pro_driver.cpp
 *
 *  Created on: Jun 2, 2017
 *      Author: Jorge Nicho
 */

#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_workbench_single_manager/dynamixel_driver.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/message_forward.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <ros/subscribe_options.h>
#include <ros/time.h>
#include <ros/timer_options.h>
#include <ros/transport_hints.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Header.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <string>

// dyanixel comm
static const int DYNAMIXEL_ID_MIN = 1;
static const int DYNAMIXEL_ID_MAX = 253;
static const int COMM_OP_DELAY = 1000; // micro seconds

// delays and polling rates
static const double MAX_MOTOR_POLLING_RATE = 80.0; // hz
static const int MAX_RW_FAILS_ALLOWED = 200;
static const double WAIT_POLL_DELAY = 0.01; // seconds
static const double MOTOR_ACTION_TIMER_DELAY = 0.1;

// tolerances
static const double MIN_POSITION_TOLERANCE = 0.01*(M_PI/180.0f); // radians
static const double DEFAULT_TIME_TOLERANCE = 1.0; // seconds
static const double DEFAULT_POSITION_TOLERANCE = (1.0)*(M_PI/180.0f); // radians

// motor config
static const int64_t MOTOR_VEL_LIMIT = 2147483647;
static const int64_t MOTOR_ACC_LIMIT = 2147483647;

// ros names
static const std::string GET_WORKBENCH_PARAMS_SERVICE = "get_workbench_parameter";
static const std::string DYNAMIXEL_MOTOR_CMD_TOPIC = "dynamixel_motor_command";
static const std::string JOINT_CMD_TOPIC = "dynamixel_cmd";
static const std::string DYNAMIXEL_JOINT_STATE_TOPIC = "dynamixel_state";
static const std::string DYNAMIXEL_DRIVER_NS = "dynamixel_driver";
static const std::string MOVE_TO_POSITION_SRV = "move_to_position";
static const std::string MOVE_TO_POSITION_ACTION = "move_motor";




double motorPosToRadians(const dynamixel_tool::DynamixelTool& motor, int64_t pos)
{
  double r = ((motor.max_radian_ - motor.min_radian_)*static_cast<double>(pos - motor.value_of_0_radian_position_))/
      static_cast<double>(motor.value_of_max_radian_position_ - motor.value_of_min_radian_position_);
  return r;
}

int64_t radiansToMotorPos(const dynamixel_tool::DynamixelTool& motor, double pos)
{
  pos = pos > motor.max_radian_ ? motor.max_radian_ : pos;
  pos = pos < motor.min_radian_ ? motor.min_radian_ : pos;

  double motor_dist = static_cast<double>(motor.value_of_max_radian_position_ - motor.value_of_min_radian_position_);
  double rad_dist = motor.max_radian_ - motor.min_radian_;

  int64_t mpos = (pos * motor_dist/rad_dist) + motor.value_of_0_radian_position_;
  return mpos;
}

double motorSpeedToAngularSpeed(const dynamixel_tool::DynamixelTool& motor, int64_t speed)
{
  return static_cast<double>(speed)/ motor.velocity_to_value_ratio_;
}

int64_t angularSpeedtoMotorSpeed(const dynamixel_tool::DynamixelTool& motor, double speed)
{
  return static_cast<int64_t>(speed * motor.velocity_to_value_ratio_);
}

bool withinPositionLimits(const dynamixel_tool::DynamixelTool& motor, double pos)
{
  if(motor.max_radian_ < pos || motor.min_radian_ > pos)
  {
    return false;
  }
  return true;
}


namespace dynamixel_operating_modes
{
  enum operating_mode: int
  {
    TORQUE_CONTROL_MODE = 0,
    VELOCITY_CONTROL_MODE = 1,
    RESERVED = 2,
    POSITION_CONTROL_MODE = 3,
    EXTENDED_POSITION_CONTROL = 4
  };
}
typedef dynamixel_operating_modes::operating_mode dynamixel_operating_mode;

bool verifyBaudRate(int br)
{
  static const std::vector<int>  BAUD_RATES = {57600, 115200, 1000000, 2000000, 3000000};
  if(std::find(BAUD_RATES.begin(),BAUD_RATES.end(),br) == BAUD_RATES.end())
  {
    std::stringstream ss;
    for(const auto& b : BAUD_RATES)
    {
      ss<<b<<" ";
    }
    ROS_ERROR("Baud rate %i is not supported, use one of the following: %s",br,ss.str().c_str());
    return false;
  }
  return true;
}


namespace dynamixel_workbench_single_manager
{

DynamixelDriver::DynamixelDriver(const std::string& device_name,
                                 const int baud_rate,
                                 const double publish_rate,
                                 const std::string& joint):
    device_name_(device_name),
    baud_rate_(baud_rate),
    publish_rate_(publish_rate),
    polling_rate_(MAX_MOTOR_POLLING_RATE),
    motor_busy_(false),
    motor_ok_(true),
    joint_name_(joint)
{


}

DynamixelDriver::~DynamixelDriver()
{
  shutdownROS();
  shutdownDynamixel();
}

bool DynamixelDriver::setup()
{
  motor_busy_ = true;
  motor_ok_ = false;

  // initializing dynamixel
  if(!initDynamixel())
  {
    shutdownDynamixel();
    return false;
  }

  // initialize ROS
  if(!initROS())
  {
    return false;
  }

  motor_busy_ = false;
  motor_ok_ = true;

  ROS_INFO("Dynamixel ready ...");
  return true;

}

bool DynamixelDriver::run()
{
  if(!setup())
  {
    return false;
  }

  return true;
}

void DynamixelDriver::setJointState(const sensor_msgs::JointState& st)
{
  std::lock_guard<std::mutex> lock(joint_st_mutex_);
  joint_st_ = st;
}

sensor_msgs::JointState DynamixelDriver::getJointState()
{
  std::lock_guard<std::mutex> lock(joint_st_mutex_);
  return joint_st_;
}

bool DynamixelDriver::initDynamixel()
{
  if(!verifyBaudRate(baud_rate_))
  {
    return false;
  }

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler and Protocol 2.0 PacketHandler
  packetHandler1_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandler2_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the dynamixel pro port(%s)!", device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the dynamixel pro port!");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
    return false;
  }

  // initializing dynamixel interface
  uint8_t dynamixel_error = 0;
  uint8_t dynamixel_id = 0;
  uint16_t dynamixel_num = 0;

  std::vector<dynamixel::PacketHandler* > packet_handlers = { packetHandler2_,packetHandler1_};
  for (dynamixel_id = DYNAMIXEL_ID_MIN; dynamixel_id < DYNAMIXEL_ID_MAX; dynamixel_id++)
  {
    for(auto& ph: packet_handlers)
    {
      if (ph->ping(portHandler_, dynamixel_id, &dynamixel_num, &dynamixel_error) == COMM_SUCCESS)
      {
        dynamixel_model_number_ = dynamixel_num;
        dynamixel_model_id_ = dynamixel_id;
        protocol_version_ = ph->getProtocolVersion();
        packetHandler_ = ph->getPacketHandler(protocol_version_);
        dynamixel_.reset(new dynamixel_tool::DynamixelTool(dynamixel_model_id_, dynamixel_model_number_, protocol_version_));

        ROS_INFO("Found Dynamixel motor [%s] with id '%u' on protocol version %f",dynamixel_->model_name_.c_str(),
                 dynamixel_id,protocol_version_);
        break;
      }
    }

    if(dynamixel_)
    {
      break;
    }
  }

  if(dynamixel_ == nullptr)
  {
    ROS_ERROR("...Failed to find dynamixel!");
    return false;
  }

  // print ctrl table
  std::stringstream ss;
  ss<<"\nControl Table:\n";
  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin();
      dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    ss<<dynamixel_->it_ctrl_->first<<": "<<dynamixel_->it_ctrl_->second<<std::endl;
  }
  ROS_DEBUG_STREAM_NAMED(DYNAMIXEL_DRIVER_NS,ss.str());

  // disabling motion
  if(!write("torque_enable", false))
  {
    return false;
  }

  // initializing operating_mode to position control
  int64_t val;
  if(read("operating_mode",&val) && val == dynamixel_operating_modes::POSITION_CONTROL_MODE)
  {
    ROS_DEBUG("Position control mode already set");
  }
  else
  {
    if(!write("operating_mode", dynamixel_operating_modes::POSITION_CONTROL_MODE))
    {
      return false;
    }
  }

  // motor limits
  if(!write("velocity_limit",MOTOR_VEL_LIMIT) && !write("acceleration_limit",MOTOR_ACC_LIMIT) )
  {
    return false;
  }

  // enabling motion
  if(!write("torque_enable", true))
  {
    return false;
  }

  // acceleration
  if(!write("goal_acceleration",0))
  {
    return false;
  }

  if(!write("goal_torque",0))
  {
    return false;
  }

  // resetting failure counter
  num_rw_fails = MAX_RW_FAILS_ALLOWED;

  return true;
}

void DynamixelDriver::shutdownDynamixel()
{
  if(dynamixel_)
  {
    write("torque_enable", false);
  }
  dynamixel_.reset();
  portHandler_->closePort();
}

bool DynamixelDriver::initROS()
{
  // service server
  workbench_param_server_ = nh_.advertiseService(GET_WORKBENCH_PARAMS_SERVICE,
                                                 &DynamixelDriver::getWorkbenchParamCallback, this);

  // move to position server
  move_to_position_server_ = nh_.advertiseService(MOVE_TO_POSITION_SRV,&DynamixelDriver::moveToPositionCallback,this);

  // dynamixel motor command subscriber
  dynamixel_command_sub_ = nh_.subscribe(DYNAMIXEL_MOTOR_CMD_TOPIC,
                                         10, &DynamixelDriver::dynamixelCommandMsgCallback,
                                         this);

  // move to position action
  move_to_position_action_server_.reset(new MoveActionServer(nh_,MOVE_TO_POSITION_ACTION,
                                                             boost::bind(&DynamixelDriver::moveToPosActionCallback,this,_1),false));
  move_to_position_action_server_->start();


  // joint state subscriber and publishers
  joint_cmd_sub_ = nh_.subscribe(JOINT_CMD_TOPIC,1,&DynamixelDriver::jointCmdCallback,this);
  joint_st_pub_ = nh_.advertise<sensor_msgs::JointState>(DYNAMIXEL_JOINT_STATE_TOPIC,1);

  // joint state publish timer
  if(publish_rate_ > polling_rate_)
  {
    ROS_WARN("The joint state publishing rate is greater the the motor polling rate");
  }
  ros::Duration period(1.0f/publish_rate_);
  auto publish_js_callback = [this](const ros::TimerEvent& evnt)
  {
    joint_st_pub_.publish(getJointState());
  };
  //  publish_joint_st_timer_ = nh_.createTimer(period,&DynamixelDriver::publishMessagesCallback,this);
  publish_joint_st_timer_ = nh_.createTimer(period,publish_js_callback);

  // poll motor timer
  poll_motor_timer_ = nh_.createTimer(ros::Duration(1.0/polling_rate_),&DynamixelDriver::pollMotor,this);

  // set name field of joint state message
  if(!joint_name_.empty())
  {
    joint_st_.name.resize(1);
    joint_st_.name.front() = joint_name_;
  }
  else
  {
    ROS_WARN("No joint name specified for JointState message");
  }

  return true;
}

void DynamixelDriver::shutdownROS()
{
  workbench_param_server_.shutdown();
  dynamixel_command_sub_.shutdown();
  joint_cmd_sub_.shutdown();
  joint_st_pub_.shutdown();
  publish_joint_st_timer_.stop();
}

void DynamixelDriver::jointCmdCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  if(motor_busy_)
  {
    ROS_ERROR("Dynamixel motor is busy, command will be ignored");
    return;
  }

  if(msg->position.empty() && msg->velocity.empty())
  {
    ROS_ERROR("Dynamixel received an invalid joint command, both the position and velocity need to be populated");
    return;
  }

  int64_t val = angularSpeedtoMotorSpeed(*dynamixel_,msg->velocity.front());
  if(!write("goal_velocity", val))
  {
    ROS_ERROR("Failed to write property '%s'","goal_velocity");
  }

  val = radiansToMotorPos(*dynamixel_,msg->position.front());
  if(!write("goal_position", val))
  {
    ROS_ERROR("Failed to write property '%s'","goal_position");
  }

}

void DynamixelDriver::publishMessagesCallback(const ros::TimerEvent& e)
{

  joint_st_pub_.publish(getJointState());
}

void DynamixelDriver::pollMotor(const ros::TimerEvent& e)
{
  // reading current position and velocity
  std::lock_guard<std::mutex> lock(joint_st_mutex_);

  int64_t vel, pos;
  sensor_msgs::JointState& js = joint_st_;
  js.velocity.resize(1);
  js.position.resize(1);

  // reading from motor
  bool success = true;
  if(!read("present_velocity", &vel) || !read("present_position", &pos))
  {
    num_rw_fails--;
    success = false;
  }
  else
  {
    num_rw_fails = MAX_RW_FAILS_ALLOWED;
  }

  // checking health
  if(num_rw_fails == 0)
  {
    ROS_ERROR("Dynamixel fail to read/write too many times, exiting");
    motor_ok_ = false;
    ros::shutdown();
  }

  if(!success)
  {
    return;
  }

  js.velocity.front() = motorSpeedToAngularSpeed(*dynamixel_,vel);

  js.position.front() = motorPosToRadians(*dynamixel_,pos);
  js.header.stamp = ros::Time::now();
}


bool DynamixelDriver::getWorkbenchParamCallback(dynamixel_workbench_msgs::GetWorkbenchParam::Request &req,
                                                   dynamixel_workbench_msgs::GetWorkbenchParam::Response &res)
{
  res.workbench_parameter.device_name = device_name_;
  res.workbench_parameter.baud_rate = portHandler_->getBaudRate();
  res.workbench_parameter.protocol_version = packetHandler_->getProtocolVersion();
  res.workbench_parameter.model_name = dynamixel_->model_name_;
  res.workbench_parameter.model_id = dynamixel_->id_;
  res.workbench_parameter.model_number = dynamixel_->model_number_;

  return true;
}

bool DynamixelDriver::stopMotor()
{
  if(!write("goal_velocity", 0))
  {
    ROS_ERROR("Failed to write property '%s'","goal_velocity");
  }

  if(!write("torque_enable", false))
  {
    return false;
  }

  if(!write("torque_enable", true))
  {
    return false;
  }

  return true;
}

bool DynamixelDriver::moveMotor(double goal_pos, double goal_speed, double pos_tolerance, double max_duration)
{
  if(motor_busy_)
  {
    ROS_ERROR("Dynamixel motor is busy");
    return false;
  }

  // blocking motor commands;
  class ScopeExit
  {
  public:
    ScopeExit(DynamixelDriver* d):
      d_(d)
    {
      d_->motor_busy_ = true;
    }
    ~ScopeExit()
    {
      d_->motor_busy_ = false;
    }

    DynamixelDriver* d_;
  };
  ScopeExit scope_exit(this);

  // resetting flag
  motor_motion_canceled_ = false;

  // variables initialization
  sensor_msgs::JointState current_js = getJointState();
  double start_pos = current_js.position.front();
  double min_completion_time = std::abs((goal_pos - start_pos)/goal_speed);
  double max_completion_time = min_completion_time + DEFAULT_TIME_TOLERANCE;


  // check limits
  if(!withinPositionLimits(*dynamixel_,goal_pos))
  {
    ROS_ERROR("Dynamixel, goal position commanded is out of bounds");
    return false;
  }

  int64_t max_motor_vel;
  const static int n_retries = 2;
  if(read("velocity_limit",&max_motor_vel, n_retries))
  {
    double max_speed = motorSpeedToAngularSpeed(*dynamixel_,max_motor_vel);
    if(max_speed < goal_speed)
    {
      ROS_ERROR("Dynamixel, goal speed is greater than the limit");
      return false;
    }
  }
  else
  {
    return false;
  }

  // setting time tolerance
  double time_tolerance;
  if(max_duration > min_completion_time)
  {
    max_completion_time = max_duration;
  }
  else
  {
    ROS_WARN("Dynamixel max_duration was too small for the specified goal, using default duration");
  }

  // setting position tolerance
  pos_tolerance = pos_tolerance < MIN_POSITION_TOLERANCE ? DEFAULT_POSITION_TOLERANCE : pos_tolerance;

  // checking if already at goal
  double pos_diff = goal_pos - start_pos  ;
  if(std::abs(pos_diff)< pos_tolerance)
  {
    ROS_WARN("Dynamixel already at goal position %f",start_pos);
    return true;
  }

  // convenience wait function
  auto wait_until_goal_funct = [&]() -> bool
  {
    ros::WallDuration wait_delay(WAIT_POLL_DELAY);
    int64_t val;
    double current_pos;
    double diff;
    double time_elapsed = 0;
    double time_diff;
    ros::WallTime start_time = ros::WallTime::now();
    bool reached_goal = false;
    double speed_cumltv = 0;
    int num_loops = 0;
    while(this->motor_ok_)
    {

      if(motor_motion_canceled_)
      {
        stopMotor();
        return false;
      }

      // updating variables
      current_pos = getJointState().position.front();
      speed_cumltv += std::abs(getJointState().velocity.front());
      diff = std::abs(goal_pos - current_pos);
      num_loops++;

      // checking goal conditions
      if(diff < pos_tolerance)
      {
        reached_goal = true;
        break;
      }

      time_diff = max_completion_time - time_elapsed ;
      if(time_diff < 0)
      {
        ROS_ERROR("Dynamixel took %f secs, which exceeded the allowed time tolerance of %f by %f, pos diff was %f",time_elapsed,
                  time_tolerance,time_diff,diff );
        break;
      }

      wait_delay.sleep();
      time_elapsed = (ros::WallTime::now() - start_time).toSec();

    }

    if(reached_goal)
    {
      ROS_DEBUG_NAMED(DYNAMIXEL_DRIVER_NS,"Dynamixel reached goal within %f secs of the expected time: %f",time_diff,max_completion_time);
    }

    ROS_DEBUG_NAMED(DYNAMIXEL_DRIVER_NS,"Dynamixel average speed is %f",speed_cumltv/num_loops);

    return reached_goal;
  };

  // check if stop requested
  if(std::abs(goal_speed) < 1e-6 )
  {
    ROS_DEBUG_NAMED(DYNAMIXEL_DRIVER_NS,"Dynamixel received a '0' velocity goal, stopping motor");
    stopMotor();
    return true;
  }

  // send speed and position to motor
  int64_t val = angularSpeedtoMotorSpeed(*dynamixel_,goal_speed);
  if(!write("goal_velocity", val, n_retries))
  {
    ROS_ERROR("Failed to write property '%s'","goal_velocity");
    return false;
  }

  val = radiansToMotorPos(*dynamixel_,goal_pos);
  if(!write("goal_position", val, n_retries))
  {
    ROS_ERROR("Failed to write property '%s'","goal_position");
    return false;
  }

  // wait until motion command is completed
  if(!wait_until_goal_funct())
  {
    return false;
  }

  return true;
}


bool DynamixelDriver::moveToPositionCallback(dynamixel_workbench_msgs::MoveToPosition::Request& req,
                            dynamixel_workbench_msgs::MoveToPosition::Response& res)
{
  double goal_pos = req.goal_state.position.front();
  double goal_speed = req.goal_state.velocity.front();
  double pos_tolerance = req.position_tolerance.empty() ? DEFAULT_POSITION_TOLERANCE : req.position_tolerance.front();
  double max_duration = req.max_duration;

  if(moveMotor(goal_pos,goal_speed,pos_tolerance,max_duration))
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }

  res.final_state = getJointState();
  return res.success;
}

void DynamixelDriver::moveToPosActionCallback(const dynamixel_workbench_msgs::MoveToPositionGoalConstPtr& goal)
{
  using namespace dynamixel_workbench_msgs;
  typedef std::shared_ptr<MoveActionServer> MoveActionServerPtr;

  // define timer callback
  auto timer_funct = [this](const ros::TimerEvent& evnt)
  {
    MoveActionServerPtr as = this->move_to_position_action_server_;

    if(as->isPreemptRequested())
    {
      ROS_DEBUG_NAMED(DYNAMIXEL_DRIVER_NS,"Dynamixel goal canceled");
      motor_motion_canceled_= true;
      return;
    }

    sensor_msgs::JointState st = getJointState();

    MoveToPositionFeedback feedback;
    feedback.current_state = getJointState();
    as->publishFeedback(feedback);
  };


  // cancel current goal
  MoveActionServerPtr as = this->move_to_position_action_server_;

  // initialize variables
  double goal_pos = goal->goal_state.position.front();
  double goal_speed = goal->goal_state.velocity.front();
  double pos_tolerance = goal->position_tolerance.empty() ? DEFAULT_POSITION_TOLERANCE : goal->position_tolerance.front();
  double max_duration = goal->max_duration;

  // create timer to monitor the goal
  ros::Timer monitor_goal_timer = nh_.createTimer(ros::Duration(MOTOR_ACTION_TIMER_DELAY),timer_funct);

  // move motor
  MoveToPositionResult res;
  bool success = moveMotor(goal_pos,goal_speed,pos_tolerance,max_duration);
  monitor_goal_timer.stop();
  if(success)
  {
    res.final_state = getJointState();
    res.success = true;
    as->setSucceeded(res);
  }
  else
  {
    res.final_state = getJointState();
    res.success = false;
    as->setAborted(res);
  }

}

void DynamixelDriver::dynamixelCommandMsgCallback(const dynamixel_workbench_msgs::DynamixelCommand::ConstPtr &msg)
{
  if (msg->addr_name == "reboot")
  {
    ROS_WARN("The '%s' command isn't supported",msg->addr_name.c_str());
    return;
  }
  else if (msg->addr_name == "factory_reset")
  {
    ROS_WARN("The '%s' command isn't supported",msg->addr_name.c_str());
    return;
  }
  else if (msg->addr_name == "baud_rate")
  {
    if(!verifyBaudRate(msg->value))
    {
      return;
    }

    shutdownROS();
    shutdownDynamixel();
    baud_rate_ = msg->value;
    initDynamixel();
    initROS();

    return;
  }
  else if (msg->addr_name == "id")
  {
    ROS_WARN("The '%s' command isn't supported",msg->addr_name.c_str());
    return;
  }
  else
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, msg->value);

    if (dynamixel_->item_->memory_type == dynamixel_tool::EEPROM)
    {
      usleep(COMM_OP_DELAY);
    }
  }
}

bool DynamixelDriver::read(const std::string& prop,int64_t *value, int retries)
{

  bool success = false;
  if(dynamixel_->ctrl_table_.count(prop)> 0)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[prop];
    do
    {
      if(readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, value))
      {
        success = true;
        break;
      }
    } while (retries-- > 0);
  }
  else
  {
    ROS_ERROR("Property '%s' was not found",prop.c_str());
  }

  return success;
}

bool DynamixelDriver::write(const std::string& prop, int64_t value, int retries)
{
  bool success = false;
  if(dynamixel_->ctrl_table_.count(prop)> 0)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[prop];
    do
    {
      if(writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, value))
      {
        success = true;
        break;
      }
    } while (retries-- > 0);
  }
  else
  {
    ROS_ERROR("Property '%s' was not found",prop.c_str());
  }

  return success;
}

bool DynamixelDriver::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t value)
{
  std::lock_guard<std::mutex> lock(dynamixel_mutex_);

  uint8_t dynamixel_error = 0;
  int dynamixel_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dynamixel_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dynamixel_error);
    usleep(COMM_OP_DELAY);
  }
  else if (length == 2)
  {
    dynamixel_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dynamixel_error);
    usleep(COMM_OP_DELAY);
  }
  else if (length == 4)
  {
    dynamixel_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dynamixel_error);
    usleep(COMM_OP_DELAY);
  }

  if (dynamixel_comm_result == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler_->printRxPacketError(dynamixel_error);
      return false;
    }
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
    return false;
  }
  return true;
}

bool DynamixelDriver::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
{
  std::lock_guard<std::mutex> lock(dynamixel_mutex_);

  uint8_t dynamixel_error = 0;
  int dynamixel_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dynamixel_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
    usleep(COMM_OP_DELAY);
  }
  else if (length == 2)
  {
    dynamixel_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
    usleep(COMM_OP_DELAY);
  }
  else if (length == 4)
  {
    dynamixel_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
    usleep(COMM_OP_DELAY);
  }

  if (dynamixel_comm_result == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler_->printRxPacketError(dynamixel_error);
      return false;
    }

    if (length == 1)
    {
      *value = value_8_bit;
      return true;

    }
    else if (length == 2)
    {
      *value = value_16_bit;
      return true;
    }
    else if (length == 4)
    {
      *value = value_32_bit;
      return true;
    }
  }
  else
  {
    //packetHandler_->printTxRxResult(dynamixel_comm_result);
    return false;
  }

  return true;
}

} /* namespace dynamixel_workbench_single_manager */

int main(int argc,char** argv)
{
  ros::init(argc,argv,"dynamixel_driver");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle ph("~");

  // reading parameters
  double publish_rate; // hz
  int baud_rate;
  std::string device_id;
  std::string joint_name;
  ph.param("publish_rate",publish_rate,50.0);
  ph.param("baud_rate",baud_rate,2000000);
  ph.param("device_id",device_id,std::string("/dev/ttyUSB0"));
  ph.param("joint_name", joint_name, std::string("dynamixel_joint"));

  spinner.start();
  dynamixel_workbench_single_manager::DynamixelDriver d(device_id, baud_rate, publish_rate, joint_name);
  if(d.run())
  {
    ros::waitForShutdown();
  }
  spinner.stop();

  return 0;
}
