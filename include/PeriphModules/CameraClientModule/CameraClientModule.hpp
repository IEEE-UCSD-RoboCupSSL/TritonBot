#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"
#include <armadillo>



class CameraClientModule : public Module {
public:    
    virtual void task(ThreadPool& threadPool) = 0;
    virtual ~CameraClientModule() {}
};


class VirtualCameraClientModule : public CameraClientModule {
public:    
    void task(ThreadPool& threadPool);
};