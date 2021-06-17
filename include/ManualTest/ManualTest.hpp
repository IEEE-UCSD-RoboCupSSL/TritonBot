#pragma once

#include "Misc/PubSubSystem/ThreadPool.hpp"
#include "Misc/PubSubSystem/Module.hpp"
#include <armadillo>
#include <CoreModules/DataCmdTypes.hpp>
#include "Misc/Utility/Common.hpp"


class ManualTest {
public:
    ITPS::FieldPublisher<bool> *ballCapStatusPubMock;

    virtual bool test(ThreadPool &threadPool) = 0;

    static bool testIntEq(const std::string& testName, int expected, int actual) {
        if (expected == actual) {
            std::cout << "[Test PASSED] test: [" << testName << "] with expected = " << expected << " and actual = "
                      << actual << std::endl;
            return true;
        }
        std::cout << "[Test FAILED] test: [" << testName << "] with expected = " << expected << " and actual = "
                  << actual << std::endl;
        return false;
    }

    static bool testDoubleEq(const std::string& testName, double expected, double actual, double precision) {
        if (((expected + precision) >= actual) && ((expected - precision) <= actual)) {
            std::cout << "[Test PASSED] test: [" << testName << "] with expected = " << expected << ", precision = " << precision << " and actual = "
                      << actual << std::endl;
            return true;
        }
        std::cout << "[Test FAILED] test: [" << testName << "] with expected = " << expected << ", precision = " << precision << " and actual = "
                  << actual << std::endl;
        return false;
    }

    static bool testStringEq(const std::string& testName, std::string& expected, std::string& actual){
        if (expected == actual) {
            std::cout << "[Test PASSED] test: [" << testName << "] with expected = " << expected << " and actual = "
                      << actual << std::endl;
            return true;
        }

        std::cout << "[Test FAILED] test: [" << testName << "] with expected = " << expected << " and actual = "
                  << actual << std::endl;
        return false;
    }

    static bool testFrameEq(const std::string& testName, ReferenceFrame expected, ReferenceFrame actual) {
        if (expected == actual) {
            std::cout << "[Test PASSED] test: [" << testName << "] with expected = " << expected << " and actual = "
                      << actual << std::endl;
            return true;
        }
        std::cout << "[Test FAILED] test: [" << testName << "] with expected = " << expected << " and actual = "
                  << actual << std::endl;
        return false;
    }

    static bool testModeEq(const std::string& testName, CtrlMode expected, CtrlMode actual) {
        if (expected == actual) {
            std::cout << "[Test PASSED] test: [" << testName << "] with expected = " << expected << " and actual = "
                      << actual << std::endl;
            return true;
        }
        std::cout << "[Test FAILED] test: [" << testName << "] with expected = " << expected << " and actual = "
                  << actual << std::endl;
        return false;
    }

    static void pauseAfterTest(){
        std::cout << "Enter any key to continue: " << std::endl;
        std::string dummy;
        std::getline(std::cin, dummy);
        std::fflush(stdin);
    }



    virtual ~ManualTest() {}
};
