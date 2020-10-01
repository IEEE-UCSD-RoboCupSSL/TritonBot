#include <gtest/gtest.h>

#include <iostream>
#include <armadillo>
#include "PubSubModule/thread_pool.hpp"
#include "Utility/boost_logger.hpp"
#include "ProtoGenerated/vFirmware_API.pb.h"
#include "Utility/systime.hpp"
#include "Utility/common.hpp"
#include "Config/config.hpp"

////////////////////////MODULES///////////////////////////
#include "EKF-Module/motion_ekf_module.hpp"
#include "EKF-Module/virtual_motion_ekf.hpp"
//////////////////////////////////////////////////////////

// We are testing modules here.
/*
 * Modules are the following:
 *  - vfirm_client
 *      - Initialize like in the main.cpp file
 *  - ControlModule
 *  - EKF-Module
 */

class vFirmClientTest: public ::testing::Test { 
public: 
   ThreadPool thread_pool(THREAD_POOL_SIZE); // pre-allocate 10 threads in a pool
   ITPS::Publisher<bool> init_sensor_pub("vfirm-client", "re/init sensors");

   boost::shared_ptr<MicroCtrlerClientModule> uc_client_module(new VFirmClient());
   uc_client_module->run(thread_pool); // runs in a separate thread

   void SetUp( ) { 
      // code here will execute just before the test ensues 
   }

   void TearDown( ) { 
      // code here will be called just after the test completes
      // ok to through exceptions from here if need be
   }
};

TEST_F( vFirmClientTest, October1 ) {
   ASSERT_EQ(2, 1);
}

TEST_F( vFirmClientTest, October2 ) {
   ASSERT_EQ(1, 1);
}

// TEST(vfirm_client, basic_test) {
//         B_Log::static_init();
//     B_Log::set_shorter_format();
//     B_Log::sink->set_filter(severity >= Info);

// /* thread pool version */
//     ThreadPool thread_pool(20); // pre-allocate 10 threads in a pool

//     VFirmClient vfirm_client_module;

//     vfirm_client_module.run(thread_pool);

//     while(1);
// }

// /*
// TEST(HelloWorld_Test, StringObjEQ)
// {
//     HelloWorld greeter;
//     std::string actual = greeter.sayHello();
//     std::string expected = "Hello, World!";
//     EXPECT_EQ(0, actual.compare("Hello, World!")) << "Expected string: \"" << expected << "\"" << std::endl << "Actual string: \"" << actual << "\"" << std::endl;
// }*/_pool.hpp"

// class vFirmClientTest: public ::testing::test { 
// public: 
//     vFirmClientTest( ) { 
//        // initialization code here
//     } 

//     void SetUp( ) { 
//        // code here will execute just before the test ensues 
//     }

//     void TearDown( ) { 
//        // code here will be called just after the test completes
//        // ok to through exceptions from here if need be
//     }
// }1

// TEST_F()

// TEST(vfirm_client, basic_test) {
//         B_Log::static_init();
//     B_Log::set_shorter_format();
//     B_Log::sink->set_filter(severity >= Info);

// /* thread pool version */
//     ThreadPool thread_pool(20); // pre-allocate 10 threads in a pool

//     VFirmClient vfirm_client_module;

//     vfirm_client_module.run(thread_pool);

//     while(1);
// }

// /*
// TEST(HelloWorld_Test, StringObjEQ)
// {
//     HelloWorld greeter;
//     std::string actual = greeter.sayHello();
//     std::string expected = "Hello, World!";
//     EXPECT_EQ(0, actual.compare("Hello, World!")) << "Expected string: \"" << expected << "\"" << std::endl << "Actual string: \"" << actual << "\"" << std::endl;
// }*/