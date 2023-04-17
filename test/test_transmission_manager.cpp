// Copyright 2023 RobeeRobotics LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include "transmission_manager/transmission_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"

using testing::SizeIs;

TEST(TransmissionManagerTest, Construct)
{
    transmission_manager::TransmissionManager t;
    ASSERT_TRUE(true);
}


TEST(TransmissionManagerTest, ScaraTransmission)
{
  // Parse transmission info
  std::string urdf_to_test = R"(
    <?xml version="1.0"?>
    <robot name="robot" xmlns="http://www.ros.org">
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
         <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
        </joint>
        <transmission name="transmission1">
          <plugin>robee_transmission_interface/ScaraTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1"/>
          <actuator name="joint2_motor" role="actuator2"/>
          <joint name="joint1" role="joint1">
            <offset>0.0</offset>
            <mechanical_reduction>2.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>-0.0</offset>
            <mechanical_reduction>-1.0</mechanical_reduction>
          </joint>
          <param name="ppr1">4000</param>
          <param name="ppr2">8000</param>
          <param name="screw_reduction">250</param>
        </transmission>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission mannager
  transmission_manager::TransmissionManager tm;
  ASSERT_TRUE(tm.init(infos[0]));
}


TEST(TransmissionManagerTest,BasicTransmission)
{
  // Parse transmission info
  std::string urdf_to_test = R"(
    <?xml version="1.0"?>
    <robot name="robot" xmlns="http://www.ros.org">
      <ros2_control name="FullSpec" type="system">
         <joint name="joint1">
                <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <state_interface name="effort"/>
         </joint>
          <transmission name="transmission_joint1">
                  <plugin>robee_transmission_interface/BasicTransmission</plugin>
                  <joint name="joint1" role="joint1">
                    <mechanical_reduction>2.0</mechanical_reduction>
                    <offset>0</offset>
                  </joint>
        </transmission>

      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission mannager
  transmission_manager::TransmissionManager tm;
  ASSERT_TRUE(tm.init(infos[0]));

  double joint_states[3] = {0.0,0.0,0.0}; //one for each state interface
  std::vector<JointHandle> jhs;
  jhs.emplace_back("joint1", "position", &joint_states[0]);
  jhs.emplace_back("joint1", "velocity", &joint_states[1]);
  jhs.emplace_back("joint1", "effort", &joint_states[2]);
  std::map<std::string, std::vector<JointHandle>>     joint_handles_states;
  joint_handles_states["joint1"] = jhs;

  double joint_cmds[2] = {0.0,0.0}; //one for each cmd interface
  std::vector<JointHandle> jhc;
  jhc.emplace_back("joint1", "position", &joint_cmds[0]);
  jhc.emplace_back("joint1", "velocity", &joint_cmds[1]);
  std::map<std::string, std::vector<JointHandle>>     joint_handles_cmds;
  joint_handles_cmds["joint1"] = jhc;


  double actuator_states[3] = {0.0,0.0,0.0}; //one for each state interface
  std::vector<ActuatorHandle> ahs;
  ahs.emplace_back("joint1", "position", &actuator_states[0]);
  ahs.emplace_back("joint1", "velocity", &actuator_states[1]);
  ahs.emplace_back("joint1", "effort", &actuator_states[2]);
  std::map<std::string, std::vector<ActuatorHandle>>     actuator_handles_states;
  actuator_handles_states["joint1"] = ahs;

  double actuator_cmds[2] = {0.0,0.0};; //one for each cmd interface
  std::vector<ActuatorHandle> ahc;
  ahc.emplace_back("joint1", "position", &actuator_cmds[0]);
  ahc.emplace_back("joint1", "velocity", &actuator_cmds[1]);
  std::map<std::string, std::vector<ActuatorHandle>>     actuator_handles_cmds;
  actuator_handles_cmds["joint1"] = ahc;

  //register handles with transmission_manager
  tm.config_states_transmissions(infos[0] , joint_handles_states, actuator_handles_states);
  tm.config_commands_transmissions(infos[0],joint_handles_cmds, actuator_handles_cmds);


  //test  setting  actuator state
  actuator_states[0] = 8;
  actuator_states[1] = 10;
 // actuator_states[2] = 12;
  ASSERT_EQ(joint_states[0],0);
  ASSERT_EQ(joint_states[1],0);
  //ASSERT_EQ(joint_states[2],0);
  tm.state_actuator_to_joint();
  //check that the joint state cmd is half the value set in actuator state
  ASSERT_EQ(joint_states[0],4);
  ASSERT_EQ(joint_states[1],5);
//  ASSERT_EQ(joint_states[2],6);


   //test  setting  command
  joint_cmds[0] = 1;
  joint_cmds[1] = 2;
  ASSERT_EQ(actuator_cmds[0],0);
  ASSERT_EQ(actuator_cmds[1],0);
  tm.cmd_joint_to_actuator();
  //check that the actuator cmd is twice the value set in joint cmd
  ASSERT_EQ(actuator_cmds[0],2);
  ASSERT_EQ(actuator_cmds[1],4);
}