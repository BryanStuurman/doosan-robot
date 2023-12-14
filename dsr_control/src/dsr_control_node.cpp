/*
 * dsr_control_node 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <signal.h>
#include "dsr_control/dsr_hw_interface.h"
#include <controller_manager/controller_manager.h>

using namespace dsr_control;

int g_nKill_dsr_control = false; 
void SigHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    
    g_nKill_dsr_control = true;
    ///usleep(1000*1000);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    bool try_reconnect = true;
    bool startup_controller_reset = true;
    unsigned int reconnects=0;
    //----- init ROS ---------------------- 
    ///ros::init(argc, argv, "dsr_control_node");
    ros::init(argc, argv, "dsr_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle private_nh("~");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);
    ros::AsyncSpinner spinner(1);
    spinner.start();


    //----- get param ---------------------
    int rate;
    private_nh.param<int>("rate", rate, 100);
    ROS_INFO("rate is %d\n", rate);
    ros::Rate r(rate);

    DRHWInterface* pArm = NULL;
    pArm = new DRHWInterface(private_nh);
    controller_manager::ControllerManager cm(pArm, private_nh);
    while(try_reconnect && (!g_nKill_dsr_control))
    {
        if(!pArm->init() ){
            ROS_ERROR("[dsr_control] Error initializing robot, init returns false");
            // return -1;
            // pArm->halt_DHI();
            break;
        }

        ros::Time last_time;
        ros::Time curr_time;
        ros::Duration dt;
        last_time = ros::Time::now();

        ROS_INFO("[dsr_control] controller_manager is updating!");

        while(ros::ok() && (false==g_nKill_dsr_control) && pArm->data_ok())
        {
            try{
                curr_time = ros::Time::now();
                dt = curr_time - last_time;
                last_time = curr_time;
                if(pArm) pArm->read(dt);
                cm.update(ros::Time::now(), dt, startup_controller_reset);
                startup_controller_reset = false;
                if(pArm) pArm->write(dt);
                r.sleep();
            }
            catch(std::runtime_error& ex)
            {
                ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
                ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
                ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
                break;
            }
        }
        ROS_WARN("[dsr_control] doosan update loop broke.");
        pArm->de_init();
        startup_controller_reset = true;
        reconnects++;
        ROS_WARN("[dsr_control] DISCONNECTED, RECONNECTING. Reconnect count at %u", reconnects);
        if (reconnects>25)
        {
            try_reconnect = false;
            ROS_FATAL("[dsr_control] DISCONNECTED, TOO MANY RECONNECTS, TERMINATING DRIVER. Reconnect count was %u", reconnects);
        }else
        {
            ROS_WARN("[dsr_control] DISCONNECTED, RECONNECTING. Reconnect count at %u", reconnects);
        }
    }

    pArm->halt_DHI();
    ROS_WARN("pre stop");
    // spinner.stop();
    ROS_WARN("post stop");

    ROS_INFO("[dsr_control] Good-bye!");

    return 0;
}