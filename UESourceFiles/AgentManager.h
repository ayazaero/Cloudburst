#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ZeroMQPlugin.h"
#include "AgentManager.generated.h"

class UAgentReceiver;
class AAircraftActor;

UCLASS()
class UE_API AAgentManager : public AActor
{
    GENERATED_BODY()

public:
    AAgentManager();
    void UpdateAircraftState(const FVector& Position, const FVector& Velocity, const FRotator& Rotation, const FVector& AngularVelocity);
    //UPROPERTY(EditAnywhere) FString AgentsFilePath;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

private:
    void CheckForNewAgents();
    void CreateAgentReceiver(const FString& AgentId, int32 Port, const FString& SubscribingAddress);

    TMap<FString, UAgentReceiver*> AgentReceivers;
    TMap<FString, AAircraftActor*> AircraftActors;

    FString AgentsFilePath;
    float CheckInterval;
    float TimeSinceLastCheck;

    UPROPERTY(EditDefaultsOnly, Category = "Spawning")
    TSubclassOf<AAircraftActor> AircraftBPClass;
};