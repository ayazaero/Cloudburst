#include "AgentReceiver.h"
#include "AircraftActor.h"
#include "JsonObjectConverter.h"
#include "Async/AsyncWork.h"

// Asynchronous Task Definition
class FProcessDataAsyncTask : public FNonAbandonableTask
{
    friend class FAutoDeleteAsyncTask<FProcessDataAsyncTask>;

private:
    UAgentReceiver* Receiver;
    FString Data;

public:
    FProcessDataAsyncTask(UAgentReceiver* InReceiver, const FString& InData)
        : Receiver(InReceiver), Data(InData)
    {}

    void DoWork()
    {
        Receiver->ParseJsonMessage(Data);
    }

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FProcessDataAsyncTask, STATGROUP_ThreadPoolAsyncTasks);
    }
};

// UAgentReceiver class implementation
UAgentReceiver::UAgentReceiver()
{
    PrimaryComponentTick.bCanEverTick = true;
    UE_LOG(LogTemp, Log, TEXT("UAgentReceiver Constructor Called"));
}

void UAgentReceiver::OnRegister()
{
    Super::OnRegister();
    // Check if PublishingPort is set correctly, otherwise log a warning and skip connection
    if (PublishingPort != 0)  // Assuming 0 is an invalid port number
    {
        FString FullAddress = "tcp://127.0.0.1:" + FString::FromInt(PublishingPort);
        UE_LOG(LogTemp, Log, TEXT("OnRegister Called: Attempting to connect with address: %s"), *FullAddress);

        bool bConnected = ZeroMQ.Connect(FullAddress);
        if (bConnected)
        {
            UE_LOG(LogTemp, Log, TEXT("Successfully connected to: %s"), *FullAddress);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to connect to: %s"), *FullAddress);
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("OnRegister Called: PublishingPort is invalid, skipping connection"));
    }
}

void UAgentReceiver::InitializeReceiver(const FString& InAgentId, int32 InPublishingPort, const FString& InSubscribingAddress)
{
    AgentId = InAgentId;
    PublishingPort = InPublishingPort;
    SubscribingAddress = InSubscribingAddress;
    // Attempt to connect now that the address has been set
    if (PublishingPort != 0)  // Assuming 0 is an invalid port number
    {
        FString FullAddress = "tcp://127.0.0.1:" + FString::FromInt(PublishingPort);
        UE_LOG(LogTemp, Log, TEXT("InitializeReceiver: Attempting to connect with address: %s"), *FullAddress);

        bool bConnected = ZeroMQ.Connect(FullAddress);
        if (bConnected)
        {
            UE_LOG(LogTemp, Log, TEXT("Successfully connected to: %s"), *FullAddress);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to connect to: %s"), *FullAddress);
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("InitializeReceiver Called: PublishingPort is invalid, skipping connection"));
    }
}

void UAgentReceiver::SetAircraftActor(AAircraftActor* InAircraftActor)
{
    AircraftActor = InAircraftActor;
}

void UAgentReceiver::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    FString ReceivedString;
    // Continuously receive messages and process each one until the queue is empty
    while (!(ReceivedString = ZeroMQ.Receive()).IsEmpty())
    {
        AsyncTask(ENamedThreads::GameThread, [this, ReceivedString]()
            {
                ParseJsonMessage(ReceivedString);
            });
    }
}


void UAgentReceiver::ReceiveData()
{
    FString Message = ZeroMQ.Receive();
    if (!Message.IsEmpty())
    {
        ParseJsonMessage(Message);
    }
}

void UAgentReceiver::ParseJsonMessage(const FString& JsonString)
{
    TSharedPtr<FJsonObject> JsonObject;
    TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

    if (FJsonSerializer::Deserialize(JsonReader, JsonObject) && JsonObject.IsValid())
    {
        FString ReceivedAgentId = JsonObject->GetStringField("agent_id");

        TArray<TSharedPtr<FJsonValue>> StateArray = JsonObject->GetArrayField("state");
        if (StateArray.Num() == 12)
        {
            // Adjust coordinates according to Unreal Engine's coordinate system
            float X = StateArray[1]->AsNumber();
            float Y = StateArray[0]->AsNumber();
            float Z = -StateArray[2]->AsNumber();

            FVector Position(X, Y, Z);
            FVector Velocity(StateArray[4]->AsNumber(), -StateArray[3]->AsNumber(), StateArray[5]->AsNumber());

            // Use incoming angles directly, without conversion from radians to degrees
            FRotator Rotation(StateArray[6]->AsNumber(), -StateArray[8]->AsNumber(), StateArray[7]->AsNumber());

            FVector AngularVelocity(StateArray[10]->AsNumber(), -StateArray[9]->AsNumber(), StateArray[11]->AsNumber());

            // Ensure updates to the game state are done on the main thread
            AsyncTask(ENamedThreads::GameThread, [this, Position, Velocity, Rotation, AngularVelocity]()
                {
                    if (AircraftActor)
                    {
                        AircraftActor->UpdateState(Position, Velocity, Rotation, AngularVelocity);
                    }
                });
        }
    }
}
