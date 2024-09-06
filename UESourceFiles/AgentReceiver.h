#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Networking.h"
#include "Dom/JsonObject.h"
#include "ZeroMQPlugin.h"
#include "AgentReceiver.generated.h"

class AAircraftActor;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UE_API UAgentReceiver : public UActorComponent
{
    GENERATED_BODY()

public:
    UAgentReceiver();

    FZeroMQWrapper ZeroMQ;

    void InitializeReceiver(const FString& InAgentId, int32 InPublishingPort, const FString& InSubscribingAddress);
    void SetAircraftActor(AAircraftActor* InAircraftActor);
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // Declare the override for OnRegister
    virtual void OnRegister() override;
    void ReceiveData();
    void ParseJsonMessage(const FString& JsonString);

private:
    FString AgentId;
    int32 PublishingPort;
    FString SubscribingAddress;
    AAircraftActor* AircraftActor;
    
};
