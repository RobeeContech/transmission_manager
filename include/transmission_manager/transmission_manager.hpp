#ifndef ROBEE_SCARA_TRANSMISSION_MANAGER__HPP_
#define ROBEE_SCARA_TRANSMISSION_MANAGER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "robee_common/robee_std.hpp"
#include "robee_common/robee_robots.hpp"

#include "robee_transmission_interface/scara_transmission.hpp"
#include "robee_transmission_interface/scara_transmission_loader.hpp"
#include "transmission_interface/transmission_loader.hpp"

#include <hardware_interface/hardware_info.hpp>
#include "pluginlib/class_loader.hpp"

#include <map>
#include <vector>
#include <string>

using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;

namespace transmission_manager
{
    
    constexpr const char * s_transmission_manager_init = "scara_transmission_manager_init";


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
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"TransmissionPluginLoader throw exception"<< ex.what());
                    return std::shared_ptr<transmission_interface::TransmissionLoader>();
                }
            }

        private:
            // must keep it alive because instance destroyers need it
            pluginlib::ClassLoader<transmission_interface::TransmissionLoader> class_loader_ = {
                "transmission_interface", "transmission_interface::TransmissionLoader"};
    };

    struct axis_data
    {
        double position[ robee_common::NUMBER_OF_SCARA_JOINTS];
        double velocity[ robee_common::NUMBER_OF_SCARA_JOINTS];
        double effort[ robee_common::NUMBER_OF_SCARA_JOINTS];

        void init(double val){
           std::fill_n(position,  robee_common::NUMBER_OF_SCARA_JOINTS, val);
           std::fill_n(velocity,  robee_common::NUMBER_OF_SCARA_JOINTS, val);
           std::fill_n(effort,  robee_common::NUMBER_OF_SCARA_JOINTS, val);
        }
    };


    template<typename T1>
    inline void copy(const T1& from, axis_data& to){
        robee_common::copy(from->position, to.position);
        robee_common::copy(from->velocity, to.velocity);
        robee_common::copy(from->effort,   to.effort);
    }

    template<typename T1>
    inline void copy(const axis_data& from, T1& to){
        robee_common::copy(from.position, to->position);
        robee_common::copy(from.velocity, to->velocity);
        robee_common::copy(from.effort,   to->effort);
    }

    struct state_data {
        axis_data joint_space;
        axis_data actuator_space;

        void init(double val){
            joint_space.init(val);
            actuator_space.init(val);
        }
    };

    struct joint_search_data{
        joint_search_data(int idx) : index(idx), found(false) 
        {}

        int index;
        bool found;
    };
    
    class scara_transmission_manager {
        public:

            state_data state;
            state_data cmd;

            scara_transmission_manager() {
                state.init(NAN);
                cmd.init(NAN);
            }

            inline void cmd_joint_to_actuator(){
                for (auto& trans : cmd_transmissions_){
                    trans->joint_to_actuator();
                }
            }

            inline void state_actuator_to_joint(){
                for (auto& trans : state_transmissions_){
                
                     try
                     {
                        
                        trans->actuator_to_joint();
                     }
                     catch(const std::exception& e)
                     {
                         RCLCPP_INFO_STREAM(rclcpp::get_logger(s_transmission_manager_init), "exception " << e.what());
                        std::cerr << e.what() << '\n';
                     }
                }
            }

            bool validate(const std::vector<hardware_interface::TransmissionInfo>& info, std::map<std::string,joint_search_data>& simple_trans_joints,int simple_transmission_count) const
            {
                if (info.size() != simple_transmission_count+1){
                    RCLCPP_FATAL(rclcpp::get_logger(s_transmission_manager_init),"info.transmissions.size()!=5");
                    return false;
                }
                bool found_ScaraTransmission = false;
                int found_simple = 0;
                
                for (auto& ti : info) {
                     if (ti.type == "transmission_interface/ScaraTransmission"){
                        if (found_ScaraTransmission){
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"found more then one scara transmission");
                            return false;
                        }
                        found_ScaraTransmission = true;
                        if (ti.joints.size() != 2){
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"scara_transmission.joints.size() != 2");
                            return false;
                        }
                        if (ti.joints[0].name != "joint_z"){
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"joint_z != scara_transmission.joints[0].name  = "<< ti.joints[0].name);
                            return false;
                        }
                        if (ti.joints[1].name != "joint_z_rot"){
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"joint_z_rot != scara_transmission.joints[1].name  = "<< ti.joints[1].name);
                            return false;
                        }
                     }
                     else if (ti.type == "transmission_interface/BasicTransmission") {
                        found_simple++;
                         if (ti.joints.size() != 1){
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"simple_transmission.joints.size() != 1");
                            return false;
                        }
                        auto it = simple_trans_joints.find(ti.joints[0].name);
                        if (it == simple_trans_joints.end())
                        {
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"unknown joint for simple transmission , name = " <<ti.joints[0].name);
                            return false;
                        }
                        if (it->second.found)
                        {
                            RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"joint for simple transmission appears twice in info , name = " <<ti.joints[0].name<<".");
                            return false;
                        }   
                        it->second.found = true;                     
                     }
                     else {
                          RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"transmission type not supported for scara robot  = "<< ti.type);
                          return false;
                     }
                }
                if (!found_ScaraTransmission){
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"did not find scara transmission");
                    return false;
                }
                 if (found_simple!=simple_transmission_count){
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"did not find "<<simple_transmission_count<<" simple transmissions");
                    return false;
                }
                return true;
            }

            bool config_transmission(const std::string& type,const std::string& name,
                                     state_data& state,
                                     const std::shared_ptr<transmission_interface::Transmission>& transmission,
                                     const std::map<std::string,joint_search_data>& simple_trans_joints) 
            {
                try
                {
                    if (type == "transmission_interface/ScaraTransmission")
                    {
                        transmission->configure({JointHandle("joint_z", hardware_interface::HW_IF_POSITION, &state.joint_space.position[2]), 
                                                JointHandle("joint_z_rot", hardware_interface::HW_IF_POSITION, &state.joint_space.position[3]),
                                                JointHandle("joint_z", hardware_interface::HW_IF_VELOCITY, &state.joint_space.velocity[2]),
                                                JointHandle("joint_z_rot", hardware_interface::HW_IF_VELOCITY, &state.joint_space.velocity[3]),
                                                JointHandle("joint_z", hardware_interface::HW_IF_EFFORT, &state.joint_space.effort[2]),
                                                JointHandle("joint_z_rot", hardware_interface::HW_IF_EFFORT, &state.joint_space.effort[3])}, 
                                            {ActuatorHandle("actuator_z", hardware_interface::HW_IF_POSITION,&state.actuator_space.position[2]),
                                                ActuatorHandle("actuator_z_rot", hardware_interface::HW_IF_POSITION, &state.actuator_space.position[3]),
                                                ActuatorHandle("actuator_z", hardware_interface::HW_IF_VELOCITY, &state.actuator_space.velocity[2]),
                                                ActuatorHandle("actuator_z_rot", hardware_interface::HW_IF_VELOCITY, &state.actuator_space.velocity[3]),
                                                ActuatorHandle("actuator_z", hardware_interface::HW_IF_EFFORT, &state.actuator_space.effort[2]),
                                                ActuatorHandle("actuator_z_rot", hardware_interface::HW_IF_EFFORT, &state.actuator_space.effort[3])});
                    
                    }
                    else if (type == "transmission_interface/BasicTransmission") 
                    {
                        int idx = simple_trans_joints.find(name)->second.index;
                         RCLCPP_INFO_STREAM(rclcpp::get_logger(s_transmission_manager_init),name<<" idx = "<<idx);
                        transmission->configure({JointHandle(name, hardware_interface::HW_IF_POSITION, &state.joint_space.position[idx]),
                                                JointHandle(name, hardware_interface::HW_IF_VELOCITY, &state.joint_space.velocity[idx]),
                                                JointHandle(name, hardware_interface::HW_IF_EFFORT, &state.joint_space.effort[idx])}, 
                                                {ActuatorHandle(name+"actuator", hardware_interface::HW_IF_POSITION,&state.actuator_space.position[idx]),
                                                ActuatorHandle(name+"actuator", hardware_interface::HW_IF_VELOCITY, &state.actuator_space.velocity[idx]),
                                                ActuatorHandle(name+"actuator", hardware_interface::HW_IF_EFFORT, &state.actuator_space.effort[idx])});
                    }  
                }
                catch(const std::exception& e)
                {
                     RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),
                     "config_transmission exception = " <<e.what());
                            return false;
                }
                return true;
            }

            bool load_transmission(const std::shared_ptr<transmission_interface::TransmissionLoader>& loader,
                                   const hardware_interface::TransmissionInfo& info,
                                   const std::map<std::string,joint_search_data>& simple_trans_joints,const bool is_state)
            {
                auto  transmission = loader->load(info);
          /*      if (transmission.get() == ) {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(s_transmission_manager_init),"TransmissionLoader failed for "<<info.type<< " for joint " << info.joints[0].name);
                    return false;
                }*/
                if (is_state) {
                    if (!config_transmission(info.type,info.joints[0].name,state,transmission,simple_trans_joints)) return false;
                    state_transmissions_.push_back(transmission);
                }
                else {
                    if (!config_transmission(info.type,info.joints[0].name,cmd,transmission,simple_trans_joints)) return false;
                    cmd_transmissions_.push_back(transmission);
                }
                return true;
            }

            bool load_transmission(const hardware_interface::TransmissionInfo& info,const std::map<std::string,joint_search_data>& simple_trans_joints)
            {
                TransmissionPluginLoader loader; 
                auto transmission_loader =loader.create(info.type);
                if (transmission_loader == nullptr)
                {
                    RCLCPP_FATAL(rclcpp::get_logger(s_transmission_manager_init),"transmission_loader =  nullptr");
                    return false;
                }
                if (!load_transmission(transmission_loader,info,simple_trans_joints, false)) return false;
                return load_transmission(transmission_loader,info, simple_trans_joints,true);
            }
            
            bool init(const std::vector<hardware_interface::TransmissionInfo>& info) {
                std::map<std::string,joint_search_data>  joint_name_2_index = { {"joint1" , joint_search_data(0)},{"joint2",joint_search_data(1)},{"joint_y_rot",joint_search_data(4)},{"joint_x_rot",joint_search_data(5)},{"joint_ptr",joint_search_data(6)}};
                if (!validate(info, joint_name_2_index,5)) return false;

                for(auto& trans_info : info) {   
                    if (!load_transmission(trans_info,joint_name_2_index)) return false;       
                }
                return true;
            }


        private:
            std::vector<std::shared_ptr<transmission_interface::Transmission>> state_transmissions_;
            std::vector<std::shared_ptr<transmission_interface::Transmission>> cmd_transmissions_;
    };       
}
#endif  // ROBEE_TRANSMISSION_MANAGER__HPP_