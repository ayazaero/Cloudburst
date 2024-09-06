#pragma once

#include "ZeroMQPlugin.h"
#include "CoreMinimal.h"
#include "Modules/ModuleManager.h" // Include the necessary header for IModuleInterface
#include "zmq.hpp"

// Ensure proper API export/import macros are used, especially if this code is in a plugin or a module

// Use ZEROMQPLUGIN_API macro for class declaration if it's part of a plugin or module
class ZEROMQPLUGIN_API FZeroMQWrapper
{
public:
    FZeroMQWrapper();
    ~FZeroMQWrapper();

    bool Connect(const FString& Address);
    bool Bind(const FString& Address);
    bool Send(const FString& Message);
    FString Receive(); // Be cautious with blocking operations

private:
    zmq::context_t* ZMQContext;
    zmq::socket_t* ZMQSocket;
};

// Declare the FZeroMQPluginModule class using the correct API macro
class ZEROMQPLUGIN_API FZeroMQPluginModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
