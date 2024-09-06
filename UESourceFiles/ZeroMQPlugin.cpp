#include "ZeroMQPlugin.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"

IMPLEMENT_MODULE(FZeroMQPluginModule, ZeroMQPlugin)

// Define the FZeroMQWrapper constructor
FZeroMQWrapper::FZeroMQWrapper()
{
    UE_LOG(LogTemp, Log, TEXT("FZeroMQWrapper Constructor Called"));
    ZMQContext = new zmq::context_t(10); // 1 is the number of I/O threads
    if (ZMQContext)
    {
        UE_LOG(LogTemp, Log, TEXT("ZMQContext successfully created"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create ZMQContext"));
    }

    ZMQSocket = new zmq::socket_t(*ZMQContext, ZMQ_SUB); // Or ZMQ_PUB, ZMQ_REQ, etc.
    if (ZMQSocket)
    {
        UE_LOG(LogTemp, Log, TEXT("ZMQSocket successfully created with type ZMQ_SUB"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create ZMQSocket"));
    }
    ZMQSocket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    UE_LOG(LogTemp, Log, TEXT("Subscribed to all messages"));
}

// Define the FZeroMQWrapper destructor
FZeroMQWrapper::~FZeroMQWrapper()
{
    UE_LOG(LogTemp, Log, TEXT("FZeroMQWrapper Destructor Called"));

    if (ZMQSocket)
    {
        ZMQSocket->close();
        UE_LOG(LogTemp, Log, TEXT("ZMQSocket closed"));
        delete ZMQSocket;
        ZMQSocket = nullptr;
        UE_LOG(LogTemp, Log, TEXT("ZMQSocket successfully deleted"));
    }

    if (ZMQContext)
    {
        ZMQContext->close();
        UE_LOG(LogTemp, Log, TEXT("ZMQContext closed"));
        delete ZMQContext;
        ZMQContext = nullptr;
        UE_LOG(LogTemp, Log, TEXT("ZMQContext successfully deleted"));
    }
}

// Define the FZeroMQWrapper::Connect method
bool FZeroMQWrapper::Connect(const FString& Address)
{
    FString ConvAddress = Address;
    UE_LOG(LogTemp, Log, TEXT("Attempting to connect to address: %s"), *ConvAddress);
    try {
        ZMQSocket->connect(TCHAR_TO_UTF8(*ConvAddress));
        UE_LOG(LogTemp, Log, TEXT("Successfully connected to address: %s"), *ConvAddress);
        return true;
    }
    catch (const zmq::error_t& e) {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Connect Error: %s"), UTF8_TO_TCHAR(e.what()));
        UE_LOG(LogTemp, Error, TEXT("Failed to connect to address: %s"), *ConvAddress);
        return false;
    }
}

// Define the FZeroMQWrapper::Bind method
bool FZeroMQWrapper::Bind(const FString& Address)
{
    FString ConvAddress = "tcp://" + Address;
    UE_LOG(LogTemp, Log, TEXT("Attempting to bind to address: %s"), *ConvAddress);
    try {
        ZMQSocket->bind(TCHAR_TO_UTF8(*ConvAddress));
        UE_LOG(LogTemp, Log, TEXT("Successfully bound to address: %s"), *ConvAddress);
        return true;
    }
    catch (const zmq::error_t& e) {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Bind Error: %s"), UTF8_TO_TCHAR(e.what()));
        UE_LOG(LogTemp, Error, TEXT("Failed to bind to address: %s"), *ConvAddress);
        return false;
    }
}

// Define the FZeroMQWrapper::Send method
bool FZeroMQWrapper::Send(const FString& Message)
{
    UE_LOG(LogTemp, Log, TEXT("Attempting to send message: %s"), *Message);
    try {
        std::string stdMessage = TCHAR_TO_UTF8(*Message);
        zmq::message_t Msg(stdMessage.begin(), stdMessage.end());
        zmq::send_result_t result = ZMQSocket->send(Msg, zmq::send_flags::none);
        if (result.has_value()) {
            UE_LOG(LogTemp, Log, TEXT("Message sent successfully"));
            return true;
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Message send failed"));
            return false;
        }
    }
    catch (const zmq::error_t& e) {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Send Error: %s"), UTF8_TO_TCHAR(e.what()));
        return false;
    }
}

// Define the FZeroMQWrapper::Receive method
FString FZeroMQWrapper::Receive()
{
    //UE_LOG(LogTemp, Log, TEXT("Attempting to receive message"));
    try {
        zmq::message_t Msg;
        zmq::recv_result_t result = ZMQSocket->recv(Msg, zmq::recv_flags::dontwait); // Use dontwait to make it non-blocking
        if (result.has_value()) {
            FString ReceivedMessage = FString(FUTF8ToTCHAR(reinterpret_cast<const ANSICHAR*>(Msg.data()), Msg.size()));
            //UE_LOG(LogTemp, Log, TEXT("Message received: %s"), *ReceivedMessage);
            return ReceivedMessage;
        }
        else {
            //UE_LOG(LogTemp, Warning, TEXT("No message received"));
            return ""; // Return empty string if no message was received
        }
    }
    catch (const zmq::error_t& e) {
        UE_LOG(LogTemp, Error, TEXT("ZMQ Receive Error: %s"), UTF8_TO_TCHAR(e.what()));
        return "";
    }
}

// Define the FZeroMQPluginModule::StartupModule method
void FZeroMQPluginModule::StartupModule()
{
    // Initialization logic here
    UE_LOG(LogTemp, Log, TEXT("ZeroMQPlugin module has started!"));

    // Example: load additional dependencies, if necessary
    FString PluginDir = IPluginManager::Get().FindPlugin(TEXT("ZeroMQPlugin"))->GetBaseDir();
    FString LibraryPath = FPaths::Combine(*PluginDir, TEXT("Source/ThirdParty/ZeroMQ/lib/libzmq-mt-gd-4_3_6.lib"));
    UE_LOG(LogTemp, Log, TEXT("ZeroMQ library path: %s"), *LibraryPath);
    if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*LibraryPath))
    {
        FPlatformProcess::GetDllHandle(*LibraryPath);
        UE_LOG(LogTemp, Log, TEXT("ZeroMQ library successfully loaded"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load ZeroMQ library"));
    }
}

// Define the FZeroMQPluginModule::ShutdownModule method
void FZeroMQPluginModule::ShutdownModule()
{
    // Cleanup logic here
    UE_LOG(LogTemp, Log, TEXT("ZeroMQPlugin module is shutting down."));
}
