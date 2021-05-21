#pragma once

#include "Misc/PubSubSystem/Module.hpp"

class CommandProcessorModule : public Module {
    public:
        virtual void task(ThreadPool& threadPool);
};
