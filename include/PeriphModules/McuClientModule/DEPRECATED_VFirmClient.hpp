#pragma once
#include "Misc/PubSubSystem/Module.hpp"
#include "CoreModules/DataCmdTypes.hpp"



class VFirmClientModule : public Module {
public:    
    void task(ThreadPool& threadPool);
};

