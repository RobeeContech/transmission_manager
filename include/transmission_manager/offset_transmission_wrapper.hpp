// Copyright 2022 RobeeRobotics LTD
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

#ifndef TRANSMISSION_INTERFACE__OFFSET_TRANSMISSION_WRAPPER_HPP_
#define TRANSMISSION_INTERFACE__OFFSET_TRANSMISSION_WRAPPER_HPP_

#include <cassert>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/accessor.hpp"
#include "transmission_interface/exception.hpp"
#include "transmission_interface/transmission.hpp"


#include "rclcpp/rclcpp.hpp"

namespace transmission_manager
{
    class OffsetTransmissionWrapper : public transmission_interface::Transmission
    {
        public:

            OffsetTransmissionWrapper( std::shared_ptr<transmission_interface::Transmission> transmission,const std::vector<double> & joint_offset) :
                wrapped_transmission_(transmission),joint_offset_(joint_offset)
            {
            }

            void configure(
                const std::vector<transmission_interface::JointHandle> & joint_handles,
                const std::vector<transmission_interface::ActuatorHandle> & actuator_handles) 
            { 
                

                std::vector<transmission_interface::JointHandle> joint_handles_wrapped(joint_handles.size());
                int j=0;
                for(int i=0;i < joint_handles.size();i++)
                {
                    if ( joint_handles[i].get_interface_name() == hardware_interface::HW_IF_POSITION)
                    {
                        joint_postion_value_.push_back(joint_handles[i].get_value());
                        joint_position_.push_back(joint_handles[i]);
                        joint_position_wrapped_.emplace_back( 
                            joint_handles[i].get_prefix_name(), joint_handles[i].get_interface_name(),&joint_postion_value_[j]);
                        joint_handles_wrapped.push_back(joint_position_wrapped_[j]);
                        j++;
                    }
                    else 
                        joint_handles_wrapped.push_back(joint_handles[i]);
                }
                wrapped_transmission_->configure(joint_handles_wrapped,actuator_handles);
            }


             void actuator_to_joint()
             { 
                wrapped_transmission_->actuator_to_joint();
                for(int i=0 ; i < joint_position_.size(); i++)
                {
                    joint_position_[i].set_value(joint_position_wrapped_[i].get_value() +  joint_offset_[i]);
                }
             }


             void joint_to_actuator()
             { 
                for(int i=0 ; i < joint_position_.size(); i++)
                {
                    joint_position_wrapped_[i].set_value(joint_position_[i].get_value() -  joint_offset_[i]);
                }
                wrapped_transmission_->joint_to_actuator();
             }


             std::size_t num_actuators() const{ return wrapped_transmission_->num_actuators(); }


             std::size_t num_joints() const{ return wrapped_transmission_->num_actuators(); }

             
             std::vector<double>& get_offsets() {return joint_offset_; }
        private:

            
            std::vector<double> joint_offset_;
            std::vector<double> joint_postion_value_;
            std::vector<transmission_interface::JointHandle> joint_position_;
            std::vector<transmission_interface::JointHandle> joint_position_wrapped_;
            std::shared_ptr<transmission_interface::Transmission> wrapped_transmission_;

    };

}
#endif //TRANSMISSION_INTERFACE__OFFSET_TRANSMISSION_WRAPPER_HPP_
