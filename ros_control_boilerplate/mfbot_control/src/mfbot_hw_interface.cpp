/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the MFBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <mfbot_control/mfbot_hw_interface.h>
#include "std_msgs/String.h"
#include <sstream>


namespace mfbot_control
{

MFBotHWInterface::MFBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("mfbot_hw_interface", "MFBotHWInterface Ready.");
}

void MFBotHWInterface::callback(const ros_control_boilerplate::UInt16Arr::ConstPtr& msg)
{

}


void MFBotHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  // Resize vectors
  joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO_NAMED(name_, "MFbotHWInterface Ready.");
}

void MFBotHWInterface::read(ros::Duration& elapsed_time)
{
//  joint_position_[0] = 0;
//  joint_position_[1] = pos_msg.data[1];
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void MFBotHWInterface::write(ros::Duration &elapsed_time, ros::Publisher &chatter_pub)
  {
    // Safety
    enforceLimits(elapsed_time);

    std_msgs::Float64MultiArray cmd_msg_float;
    ros_control_boilerplate::UInt16Arr cmd_msg;
    std_msgs::Float64MultiArray position_float;
    float curPosStepper;
    float newPosStepper;
    float stepsStepper;

//  push data into data blob
    std::vector<double>::const_iterator itr, end(joint_position_command_.end());
    for(itr = joint_position_command_.begin(); itr!= end; ++itr) {
        //cout<<*itr<<endl;
        cmd_msg_float.data.push_back(*itr);
    }/**/

    while (cmd_msg_float.data[0] < 0){
      cmd_msg_float.data[0] = cmd_msg_float.data[0] + 2 * 3.14159265359;
    }
    cmd_msg_float.data[0] = (fmod((cmd_msg_float.data[0] * 180.0 / 3.14159265359), 360.0));
    cmd_msg_float.data[1] = cmd_msg_float.data[1] * 180.0 / 3.14159265359 + (90.0);


    std::vector<double>::const_iterator itr2, end2(joint_position_.end());
    for(itr2 = joint_position_.begin(); itr2!= end2; ++itr2) {
        //cout<<*itr<<endl;
        position_float.data.push_back(*itr2);
    }/**/

    while (position_float.data[0] < 0){
      curPosStepper = position_float.data[0] + 2 * 3.14159265359;
    }

    curPosStepper = 1; // (fmod((position_float.data[0] * 180.0 / 3.14159265359), 360.0));


    newPosStepper = cmd_msg_float.data[0];


    if(abs(newPosStepper - curPosStepper)<abs(newPosStepper-curPosStepper-360)){
      stepsStepper = (newPosStepper - curPosStepper) /360 * 2048;
    }else{
      stepsStepper = (newPosStepper - curPosStepper - 360) /360 * 2048;
    }


    cmd_msg.data.resize(2);
    cmd_msg.data[0] = static_cast<unsigned int>(stepsStepper);
    cmd_msg.data[1] = static_cast<unsigned int>(cmd_msg_float.data[1]);

    chatter_pub.publish(cmd_msg);

    // ----------------------------------------------------
    //
    // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
    //
    // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
    // VELOCITY FROM POSITION WITH SMOOTHING, SEE
    // sim_hw_interface.cpp IN THIS PACKAGE
    //
    // DUMMY PASS-THROUGH CODE
//    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
//     joint_position_[joint_id] += joint_position_command_[joint_id];
    //END DUMMY CODE
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
        const double max_delta_pos = joint_velocity_limits_[joint_id] * elapsed_time.toSec();

        // Move all the states to the commanded set points at max velocity
        p_error_ = joint_position_command_[joint_id] - joint_position_[joint_id];

        const double delta_pos = std::max(std::min(p_error_, max_delta_pos), -max_delta_pos);
        joint_position_[joint_id] += delta_pos;

        // Bypass max velocity p controller:
        //joint_position_[joint_id] = joint_position_command_[joint_id];

        // Calculate velocity based on change in positions
        if (elapsed_time.toSec() > 0)
        {
          joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
        }
        else
          joint_velocity_[joint_id] = 0;

        // Save last position
        joint_position_prev_[joint_id] = joint_position_[joint_id];
    }
  }

void MFBotHWInterface::enforceLimits(ros::Duration & period)
  {
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
  }

} // namespace
