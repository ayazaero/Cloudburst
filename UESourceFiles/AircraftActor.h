#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AircraftActor.generated.h"

UCLASS()
class UE_API AAircraftActor : public AActor
{
    GENERATED_BODY()

public:
    AAircraftActor();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

    void UpdateState(const FVector& Position, const FVector& Velocity, const FRotator& Rotation, const FVector& AngularVelocity);

private:
    UPROPERTY(VisibleAnywhere)
    UStaticMeshComponent* AircraftMesh;
};