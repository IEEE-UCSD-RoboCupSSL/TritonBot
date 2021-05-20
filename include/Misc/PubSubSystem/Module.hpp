/* Author: Hongtao Zhang */ 
#pragma once
 
#include <iostream>
#include "PubSub.hpp"
#include "Observer.hpp"
#include "ThreadPool.hpp"

class Module {
    public:
        Module() {

        }
        ~Module() {

        }

        virtual void task() {}
        virtual void task(ThreadPool& threadPool) {}
        
        //======================Create New Thread Version=================================//
        /* create a new thread and run the module in that thread */
        void run() {
            mthread = boost::shared_ptr<boost::thread>(
                new boost::thread(boost::bind(&Module::task, this))
            );
        }
        /* don't use this method if the threadpool version of Module::run() was used */
        void idle() {
            mthread->yield(); 
        }

        /* don't use this method if the threadpool version of Module::run() was used */
        void join() {
            mthread->join();
        }
        //================================================================================//




        //============================Thread Pool Version=================================//
        /* run the module as a task to be queued for a thread pool*/
        void run(ThreadPool& threadPool) {
            threadPool.execute(boost::bind(&Module::task, this, boost::ref(threadPool)));
        }
        //================================================================================//

    private:
        boost::shared_ptr<boost::thread> mthread;

};
