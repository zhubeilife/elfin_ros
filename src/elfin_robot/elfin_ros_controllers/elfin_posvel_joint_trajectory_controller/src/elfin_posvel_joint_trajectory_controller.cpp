/*
Created on Thu Oct 25 10:38 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include <pluginlib/class_list_macros.hpp>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <elfin_posvel_joint_trajectory_controller/posvel_hw_interface_adapter.h>

namespace elfin_pos_vel_controllers {

    typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                   elfin_hardware_interface::PosVelTrqJointInterface>
    JointTrajectoryController;

}

PLUGINLIB_EXPORT_CLASS(elfin_pos_vel_controllers::JointTrajectoryController, controller_interface::ControllerBase)
