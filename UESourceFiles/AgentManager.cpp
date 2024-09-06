#include "AgentManager.h"
#include "AgentReceiver.h"
#include "AircraftActor.h"
#include "ZeroMQPlugin.h"
#include "Misc/FileHelper.h"

AAgentManager::AAgentManager()
{
    PrimaryActorTick.bCanEverTick = true;

    static ConstructorHelpers::FClassFinder<AAircraftActor> BPClass(TEXT("/Game/SimBlank/Blueprints/BP_AircraftActor"));
    if (BPClass.Class != nullptr)
    {
        AircraftBPClass = BPClass.Class;
    }

    AgentsFilePath = FPaths::ProjectContentDir() + "../../agents.txt";
    CheckInterval = 1.0f; // Check every second
}

void AAgentManager::BeginPlay()
{
    Super::BeginPlay();
}

void AAgentManager::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    TimeSinceLastCheck += DeltaTime;
    if (TimeSinceLastCheck >= CheckInterval)
    {
        CheckForNewAgents();
        TimeSinceLastCheck = 0.0f;
    }
}

void AAgentManager::CheckForNewAgents()
{
    TArray<FString> Lines;
    FFileHelper::LoadFileToStringArray(Lines, *AgentsFilePath);

    for (const FString& Line : Lines)
    {
        TArray<FString> Parts;
        if (Line.ParseIntoArray(Parts, TEXT(","), true) == 3)
        {
            FString AgentId = Parts[0];
            FString PublishingAddress = Parts[1];  // Format: tcp://127.0.0.1:5557
            FString SubscribingAddress = Parts[2]; // Format: tcp://127.0.0.1:5560
            FString RightSide;
            //UE_LOG(LogTemp, Log, TEXT("Publishing Address: %s"), PublishingAddress);

            if (PublishingAddress.Split(TEXT(":"), nullptr, &RightSide))
            {
                FString PortStr;
                if (RightSide.Split(TEXT(":"), nullptr, &PortStr))
                {
                    int32 Port = FCString::Atoi(*PortStr);
                    //UE_LOG(LogTemp, Log, TEXT("Converted Port: %d"), Port);

                    if (!AgentReceivers.Contains(AgentId))
                    {
                        CreateAgentReceiver(AgentId, Port, SubscribingAddress);
                    }
                }
            }
            // Extract port number from PublishingAddress
            /*FString PortStr;
            if (PublishingAddress.Split(TEXT(":"), nullptr, &PortStr))
            {
                UE_LOG(LogTemp, Log, TEXT("Extracted Port String: %s"), *PortStr);
                int32 Port = FCString::Atoi(*PortStr);
                UE_LOG(LogTemp, Log, TEXT("Converted Port: %d"), Port);

                if (!AgentReceivers.Contains(AgentId))
                {
                    CreateAgentReceiver(AgentId, Port, SubscribingAddress);
                }
            }*/
        }
    }
}

void AAgentManager::CreateAgentReceiver(const FString& AgentId, int32 Port, const FString& SubscribingAddress)
{
    UAgentReceiver* NewReceiver = NewObject<UAgentReceiver>(this);
    NewReceiver->RegisterComponent();  // This registers the component with the Unreal Engine runtime.
    NewReceiver->InitializeReceiver(AgentId, Port, SubscribingAddress);
    AgentReceivers.Add(AgentId, NewReceiver);

    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    if (AircraftBPClass != nullptr)
    {
        AAircraftActor* NewAircraft = GetWorld()->SpawnActor<AAircraftActor>(AircraftBPClass, FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
        if (NewAircraft)
        {
            AircraftActors.Add(AgentId, NewAircraft);
            NewReceiver->SetAircraftActor(NewAircraft);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to spawn AircraftActor for agent %s"), *AgentId);
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("AircraftBlueprintClass is null. Make sure it's set correctly."));
    }
}