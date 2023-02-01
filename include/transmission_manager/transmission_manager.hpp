#ifndef TRANSMISSION_MANAGER__HPP_
#define TRANSMISSION_MANAGER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "transmission_interface/transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include <hardware_interface/hardware_info.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include <map>
#include <set>
#include <vector>
#include <string>

using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;

namespace transmission_manager
{
    
    constexpr const char * s_transmission_manager = "transmission_manager";


    class TransmissionPluginLoader
    {
        public:
            std::shared_ptr<transmission_interface::TransmissionLoader> create(const std::string & type)
            {
                try
                {
                    return class_loader_.createUniqueInstance(type);
                }
                catch (std::exception & ex)
                {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),"TransmissionPluginLoader throw exception"<< ex.what());
                    return std::shared_ptr<transmission_interface::TransmissionLoader>();
                }
            }

        private:
            // must keep it alive because instance destroyers need it
            pluginlib::ClassLoader<transmission_interface::TransmissionLoader> class_loader_ = {
                "transmission_interface", "transmission_interface::TransmissionLoader"};
    };


    class TransmissionManager {
        public:

            inline void cmd_joint_to_actuator();

            inline void state_actuator_to_joint();
         
            bool init(const hardware_interface::HardwareInfo& info);

            void config_states_transmissions(
                const hardware_interface::HardwareInfo&             info,
                std::map<std::string,std::vector<JointHandle>>&     joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>& actuator_handles);

            void config_commands_transmissions(
                const hardware_interface::HardwareInfo&             info,
                std::map<std::string,std::vector<JointHandle>>&     joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>& actuator_handles);

        private:

            bool validate(const hardware_interface::HardwareInfo& info);

            void config_transmission(
                std::vector<std::shared_ptr<transmission_interface::Transmission>>& transmissions,
                const hardware_interface::HardwareInfo&                             info,
                std::map<std::string,std::vector<JointHandle>>&                             joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>&                         actuator_handles);

            bool load_transmission(const std::shared_ptr<transmission_interface::TransmissionLoader>& loader,
                                   const hardware_interface::TransmissionInfo& info,
                                   const bool is_state);
         

            bool load_transmission(const hardware_interface::TransmissionInfo& info);
           
            
            std::vector<std::shared_ptr<transmission_interface::Transmission>> state_transmissions_;
            std::vector<std::shared_ptr<transmission_interface::Transmission>> cmd_transmissions_;
    };       
}
#endif  // TRANSMISSION_MANAGER__HPP_