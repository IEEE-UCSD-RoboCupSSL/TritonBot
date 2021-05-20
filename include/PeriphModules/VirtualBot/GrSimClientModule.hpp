#pragma once
#include "Misc/PubSubSystem/Module.hpp"


class GrSimClientModule : public Module {
    public:        
        void task(ThreadPool& threadPool);
};
