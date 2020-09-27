#include <iostream>
#include <gtest/gtest.h>

#include "PubSubModule/thread_pool.hpp"
#include "MicroCtrlerClient/vfirm_client.hpp"
#include "Utility/boost_logger.hpp"


// class myTestFixture1: public ::testing::test { 
// public: 
//     myTestFixture1( ) { 
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
// }*/_pool.hpp"
#include "MicroCtrlerClient/vfirm_client.hpp"
#include "Utility/boost_logger.hpp"


// class myTestFixture1: public ::testing::test { 
// public: 
//     myTestFixture1( ) { 
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