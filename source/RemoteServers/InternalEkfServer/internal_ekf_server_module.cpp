#include "RemoteServers/InternalEkfServer/internal_ekf_server_module.hpp"

#include <string>
#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Utility/systime.hpp"
#include "PubSubSystem/thread_pool.hpp"
#include "ProtoGenerated/RemoteAPI.pb.h"
#include "Utility/boost_logger.hpp"
#include "Config/config.hpp"
#include "Utility/common.hpp"
#include "EKF-Module/motion_ekf_module.hpp"

using namespace boost;

void InternalEkfServer::task(ThreadPool& thread_pool)
{
    B_Log logger;
    logger.add_tag("Internal EKF Server Module");
    logger(Info) << "\033[0;32m Thread Started \033[0m";

    RobotInternalData data;

    /*** TCP server setup***/
    asio::io_service io_service;
    asio::ip::udp::endpoint endpoint_to_send(asio::ip::udp::v4(), EKF_SERVER_PORT);
    asio::ip::udp::socket socket(io_service, endpoint_to_send);
    
    std::string write_buf;

    /*** Publisher setup ***/
    
    ITPS::NonBlockingSubscriber<MotionEKF_Module::MotionData> ekf_data_sub("MotionEKF", "MotionData");
    
    try{
        ekf_data_sub.subscribe(DEFAULT_SUBSCRIBER_TIMEOUT);
    }
    catch(std::exception& e){
        B_Log logger;
        logger.add_tag("[internal_ekf_server_module.cpp]");
        logger.log(Error, e.what());
        while(1);
    }

    logger.log(Info, "Server Started on Port Number:" + repr(EKF_SERVER_PORT) 
                + ", Sending Internal EKF Data");

    try{

        while(true){
            /*** Building protobuf object ***/
            MotionEKF_Module::MotionData latest_data = ekf_data_sub.latest_msg();
            Vec2D zero_vec_vel = Vec2D();
            Vec2D zero_vec_dis = Vec2D();

            // TODO: Check if vec access is correct
            zero_vec_vel.set_x(latest_data.trans_vel(0));
            zero_vec_vel.set_y(latest_data.trans_vel(1));
            zero_vec_dis.set_x(latest_data.trans_disp(0));
            zero_vec_dis.set_y(latest_data.trans_disp(1));
            data.set_allocated_trans_vel(&zero_vec_vel);
            data.set_allocated_trans_disp(&zero_vec_dis);
            data.set_rotat_disp(latest_data.rotat_disp);
            data.set_rotat_vel(latest_data.rotat_vel);

            /*** Serialize protobuf object and send ***/
            data.SerializeToString(&write_buf);
            

            // TODO: Do we still need this?
            //write_buf += "\n";
            
            // TODO: Netcat is not printing stuff out.
            socket.send_to(asio::buffer(write_buf), endpoint_to_send);
            /*** Release and clear protobuf object ***/
            data.release_trans_disp();
            data.release_trans_vel();
            data.Clear();

        }
    }
    catch(std::exception& e)
    {
        logger.log(Error, "[Exception]" + std::string(e.what()));
    }
}