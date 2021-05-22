#include "ManualTest/TestRunner.hpp"
#include "PeriphModules/RemoteServers/TcpReceiveModule.hpp"



ManualTest* TestRunner::findTest(std::string testName) {
    auto iter = tests_map.find(testName);
    if(iter == tests_map.end()) return nullptr;
    else return iter->second;
}


void TestRunner::printAllAvailableTests() {
    for(auto it = tests_map.begin(); it != tests_map.end(); it++) {
        std::cout << "- " << it->first << std::endl;
    }
}


void TestRunner::run(ThreadPool& threadPool) {
    /* Interactive Tests requires stdin/stdout, so std::cout/cin is used rather than logger */

    bool result;
    std::string testName = "";
    ManualTest *testObject;

    // run the tcp module first in order to accept connection from TritonSoccerAI.java
    ITPS::FieldPublisher<bool> placeHolder("From:BallCaptureModule", "isDribbled", false); 
    std::unique_ptr<TcpReceiveModule> tcpReceiveModule(new TcpReceiveModule());
    tcpReceiveModule->run(threadPool);

    while(true) {
        std::cout << "Available Tests:" << std::endl;
        printAllAvailableTests();
        std::cout << ">> ENTER TEST NAME:" << std::endl;
        
        std::getline(std::cin, testName);
        if(testName == "quit") break;

        testObject = findTest(testName);
        if(testObject == nullptr) {
            result = false;
        } else {
            result = testObject->test(threadPool);
        }

        std::cout << (result ? "Test Success" : "Test Failed") << std::endl;
    }
}

