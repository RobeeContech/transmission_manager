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

    /**
     * @brief class to handle the loading of transmission plugins
     * 
     */
    class TransmissionPluginLoader
    {
        public:
        /**
         * @brief create a transmission plugin instance
         * 
         * @param type 
         * @return std::shared_ptr<transmission_interface::TransmissionLoader> 
         */
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

    /**
     * @brief class to manage the transmission of a joint
     * 
     */
    class TransmissionManager {
        public:
            /**
             * @brief joint to actuator
             * 
             */
            inline void cmd_joint_to_actuator()
            {
                for (auto& trans : cmd_transmissions_)
                {
                    trans->joint_to_actuator();
                }
            }
            /**
             * @brief actuator to joint
             * 
             */
            inline void state_actuator_to_joint()
            {
                for (auto& trans : state_transmissions_)
                {
                    trans->actuator_to_joint();
                }
            }
            /**
             * @brief init the transmission and load it
             * 
             * @param info 
             * @return true 
             * @return false 
             */
            bool init(const hardware_interface::HardwareInfo& info)
            {
                if (!validate(info)) return false;

                for(auto& trans_info : info.transmissions) {   
                    if (!load_transmission(trans_info)) return false;       
                }
                return true;
            }
            /**
             * @brief config actuator to joint
             * 
             * @param info 
             * @param joint_handles 
             * @param actuator_handles 
             */
            void config_states_transmissions(
                const hardware_interface::HardwareInfo&             info,
                std::map<std::string,std::vector<JointHandle>>&     joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>& actuator_handles)
            {
                config_transmission(state_transmissions_, info, joint_handles, actuator_handles);
            }
            /**
             * @brief config joint to actuator
             * 
             * @param info 
             * @param joint_handles 
             * @param actuator_handles 
             */
            void config_commands_transmissions(
                const hardware_interface::HardwareInfo&             info,
                std::map<std::string,std::vector<JointHandle>>&     joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>& actuator_handles)
            {
                config_transmission(cmd_transmissions_, info, joint_handles, actuator_handles);
            }

        private:
            /**
             * @brief validate the transmission plugin
             * 
             * @param info 
             * @return true 
             * @return false 
             */
            bool validate(const hardware_interface::HardwareInfo& info)
            {
                std::set<std::string> joints_in_transmissions;
                for (auto& ti : info.transmissions) 
                {
                    if (ti.joints.size()==0)
                    {
                        RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),"validate failed: found transmission "<< ti.name<<  " with zero joints");
                        return false;
                    }
                    for (auto& ji : ti.joints)
                    {
                        //in case find joint already in other transmission
                        if (joints_in_transmissions.find(ji.name) !=  joints_in_transmissions.end())
                        {
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),"validate failed: found joint " << ji.name<< " in more then one transmission!");
                            return false;
                        }
                        joints_in_transmissions.insert(ji.name);
                    }
                }

                //check that number of  joints are the same as number of joints in transmission
                if (joints_in_transmissions.size() !=  info.joints.size())
                {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),
                        "validate failed: "<< joints_in_transmissions.size() <<
                        " = joints_in_transmissions.size() != info.joints.size() = " << info.joints.size());
                    return false;
                }

                // check that each joint in info.joints is part of one transmission
                for (auto& ji : info.joints) 
                {
                    if (joints_in_transmissions.find(ji.name) ==  joints_in_transmissions.end())
                    {
                        RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),
                            "validate failed: joint " << ji.name<< " from info.joints was not found in any transmission!");
                        return false;
                    }
                }
                return true;
            }
            /**
             * @brief config the transmission
             * 
             * @param transmissions 
             * @param info 
             * @param joint_handles 
             * @param actuator_handles 
             */
            void config_transmission(
                std::vector<std::shared_ptr<transmission_interface::Transmission>>& transmissions,
                const hardware_interface::HardwareInfo&                             info,
                std::map<std::string,std::vector<JointHandle>>&                             joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>&                         actuator_handles)
            {
                for (long unsigned int t=0 ; t <transmissions.size(); t++)
                {
                    std::vector<JointHandle>    joint_handle    = joint_handles[info.transmissions[t].joints[0].name];
                    std::vector<ActuatorHandle> actuator_handle = actuator_handles[info.transmissions[t].joints[0].name];
                    for(long unsigned int j=1; j<info.transmissions[t].joints.size(); j++ )
                    {
                        auto& jh = joint_handles[info.transmissions[t].joints[j].name];
                        auto& ah = actuator_handles[info.transmissions[t].joints[j].name];
                        std::copy(jh.begin(), jh.end(), std::back_inserter(joint_handle));
                        std::copy(ah.begin(), ah.end(), std::back_inserter(actuator_handle));
                    }
                    transmissions[t]->configure(joint_handle, actuator_handle);
                }
            }
            /**
             * @brief load the transmission
             * 
             * @param loader 
             * @param info 
             * @param is_state 
             * @return true 
             * @return false 
             */
            bool load_transmission(const std::shared_ptr<transmission_interface::TransmissionLoader>& loader,
                                   const hardware_interface::TransmissionInfo& info,
                                   const bool is_state)
             {
                std::shared_ptr<transmission_interface::Transmission>  transmission(loader->load(info));
                if (!transmission ) {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),"TransmissionLoader.load failed for "<<info.type<< " for joint " << info.joints[0].name);
                    return false;
                }
                if (is_state) state_transmissions_.push_back(transmission);
                else          cmd_transmissions_.push_back(transmission);
                return true;
            }   
         
            /**
             * @brief load the transmission
             * 
             * @param info 
             * @return true 
             * @return false 
             */
            bool load_transmission(const hardware_interface::TransmissionInfo& info)
            {
                TransmissionPluginLoader loader; 
                auto transmission_loader =loader.create(info.type);
                if (transmission_loader == nullptr)
                {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager),"loader.create( failed for " <<info.type << 
                                " transmission_loader =  nullptr");
                    return false;
                }
                return load_transmission(transmission_loader,info, false) && load_transmission(transmission_loader,info,true);
            }
           
            
            std::vector<std::shared_ptr<transmission_interface::Transmission>> state_transmissions_;
            std::vector<std::shared_ptr<transmission_interface::Transmission>> cmd_transmissions_;
    };       
}
#endif  // TRANSMISSION_MANAGER__HPP_