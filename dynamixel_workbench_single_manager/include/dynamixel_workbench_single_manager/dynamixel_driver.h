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
 * dynamixel_driver.h
 *
 *  Created on: Jun 2, 2017
 *      Author: Jorge Nicho
 *
 */

#ifndef INCLUDE_DYNAMIXEL_DRIVER_H_
#define INCLUDE_DYNAMIXEL_DRIVER_H_

#include <ros/ros.h>
#include <memory>
#include <mutex>
#include <atomic>

#include <sensor_msgs/JointState.h>
#include <dynamixel_workbench_msgs/MoveToPosition.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/WorkbenchParam.h>
#include <dynamixel_workbench_msgs/GetWorkbenchParam.h>
#include <dynamixel_workbench_msgs/MoveToPositionAction.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <actionlib/server/simple_action_server.h>

namespace dynamixel_workbench_single_manager
{

class DynamixelDriver
{
public:
  DynamixelDriver(const std::string& device_name,
                  const int baud_rate,
                  const double publish_rate,
                  const std::string& joint);

  virtual ~DynamixelDriver();

  bool run();

protected:

 // ROS NodeHandle
 ros::NodeHandle nh_;

 // ROS interface
 ros::Subscriber dynamixel_command_sub_;
 ros::Subscriber joint_cmd_sub_;
 ros::Publisher joint_st_pub_;
 ros::ServiceServer workbench_param_server_;
 ros::ServiceServer move_to_position_server_;
 ros::Timer publish_joint_st_timer_;
 ros::Timer poll_motor_timer_;
 typedef actionlib::SimpleActionServer<dynamixel_workbench_msgs::MoveToPositionAction> MoveActionServer;
 std::shared_ptr< MoveActionServer > move_to_position_action_server_;

 // others
 double polling_rate_;
 double publish_rate_; // hz

 // Motor Info
 std::string device_name_;
 int baud_rate_;
 float protocol_version_;
 uint16_t dynamixel_model_number_;
 uint8_t dynamixel_model_id_;

 // Motor read write
 std::mutex dynamixel_mutex_;
 std::shared_ptr<dynamixel_tool::DynamixelTool> dynamixel_;
 dynamixel::PortHandler *portHandler_;
 dynamixel::PacketHandler *packetHandler_;
 dynamixel::PacketHandler *packetHandler1_;
 dynamixel::PacketHandler *packetHandler2_;

 // Motor state
 std::atomic<bool> motor_motion_canceled_;
 std::atomic<bool> motor_busy_;
 std::mutex joint_st_mutex_;
 sensor_msgs::JointState joint_st_;
 std::string joint_name_;

 // health
 std::atomic<bool> motor_ok_;
 int num_rw_fails;

protected:

 // workflow methods
 bool setup();
 bool initDynamixel();
 bool initROS();
 void shutdownROS();
 void shutdownDynamixel();

 void setJointState(const sensor_msgs::JointState& st);
 sensor_msgs::JointState getJointState();


 // support methods
 bool read(const std::string& prop,int64_t *value, int retries = 0);
 bool write(const std::string& prop,int64_t value, int retries = 0);
 bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t value);
 bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);

 bool moveMotor(double goal_pos,double goal_speed, double pos_tolerance, double max_duration);
 bool stopMotor();

 // callbacks
 void pollMotor(const ros::TimerEvent& e);
 void dynamixelCommandMsgCallback(const dynamixel_workbench_msgs::DynamixelCommand::ConstPtr &msg);
 bool getWorkbenchParamCallback(dynamixel_workbench_msgs::GetWorkbenchParam::Request &req,
                                dynamixel_workbench_msgs::GetWorkbenchParam::Response &res);
 bool moveToPositionCallback(dynamixel_workbench_msgs::MoveToPosition::Request& req,
                             dynamixel_workbench_msgs::MoveToPosition::Response& res);
 void moveToPosActionCallback(const dynamixel_workbench_msgs::MoveToPositionGoalConstPtr& goal);
 void jointCmdCallback(const sensor_msgs::JointStateConstPtr &msg);
 void publishMessagesCallback(const ros::TimerEvent& e);


};

} /* namespace dynamixel_workbench_single_manager */

#endif /* INCLUDE_DYNAMIXEL_DRIVER_H_ */
