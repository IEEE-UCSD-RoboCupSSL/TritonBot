//
// Created by samuelluohaoen on 6/14/2021.
//
#pragma once
#ifndef TRITONBOT_BALLCAPTEST_HPP
#define TRITONBOT_BALLCAPTEST_HPP


#include <ManualTest/ManualTest.hpp>
#include <Config/Config.hpp>

class BallCapTest : public ManualTest {
public:
    BallCapTest(Config&);

    bool test(ThreadPool &threadPool) override;

    ~BallCapTest() override;

protected:
    Config config;

};


#endif //TRITONBOT_BALLCAPTEST_HPP
