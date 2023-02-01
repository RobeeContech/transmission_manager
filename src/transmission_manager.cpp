
#include "transmission_manager/transmission_manager.hpp"


namespace transmission_manager
{
    inline void TransmissionManager::cmd_joint_to_actuator(){
        for (auto& trans : cmd_transmissions_){
            trans->joint_to_actuator();
        }
    }

    inline void TransmissionManager::state_actuator_to_joint(){
        for (auto& trans : state_transmissions_){
          trans->actuator_to_joint();
        }
    }

    //check that for each joint there is exactly one transmission
    bool TransmissionManager::validate(const hardware_interface::HardwareInfo& info)
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

   

    bool TransmissionManager::load_transmission(
                            const std::shared_ptr<transmission_interface::TransmissionLoader>& loader,
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

    bool TransmissionManager::load_transmission(
        const hardware_interface::TransmissionInfo& info)
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
    
    bool TransmissionManager::init(const hardware_interface::HardwareInfo& info)    {
        if (!validate(info)) return false;

        for(auto& trans_info : info.transmissions) {   
            if (!load_transmission(trans_info)) return false;       
        }
        return true;
    }

    void TransmissionManager::config_transmission(
                std::vector<std::shared_ptr<transmission_interface::Transmission>>& transmissions,
                const hardware_interface::HardwareInfo&                             info,
                std::map<std::string,std::vector<JointHandle>>&                     joint_handles, 
                std::map<std::string, std::vector<ActuatorHandle>>&                 actuator_handles)
    {
        for (int t=0 ; t <transmissions.size(); t++)
        {
            std::vector<JointHandle>    joint_handle    = joint_handles[info.transmissions[t].joints[0].name];
            std::vector<ActuatorHandle> actuator_handle = actuator_handles[info.transmissions[t].joints[0].name];
            for(int j=1; j<info.transmissions[t].joints.size(); j++ )
            {
                auto& jh = joint_handles[info.transmissions[t].joints[j].name];
                auto& ah = actuator_handles[info.transmissions[t].joints[j].name];
                std::copy(jh.begin(), jh.end(), std::back_inserter(joint_handle));
                std::copy(ah.begin(), ah.end(), std::back_inserter(actuator_handle));
            }
            transmissions[t]->configure(joint_handle, actuator_handle);
        }
    }

    void TransmissionManager::config_states_transmissions(
        const hardware_interface::HardwareInfo&     info,
        std::map<std::string,std::vector<JointHandle>>&     joint_handles, 
        std::map<std::string, std::vector<ActuatorHandle>>& actuator_handles)
    {
        config_transmission(state_transmissions_, info, joint_handles, actuator_handles);
    }

    void TransmissionManager::config_commands_transmissions(
        const hardware_interface::HardwareInfo&      info,
        std::map<std::string,std::vector<JointHandle>>&     joint_handles, 
        std::map<std::string, std::vector<ActuatorHandle>>& actuator_handles)
    {
        config_transmission(cmd_transmissions_, info, joint_handles, actuator_handles);
    }

    

}