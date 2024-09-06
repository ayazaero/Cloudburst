#include "AircraftActor.h"

AAircraftActor::AAircraftActor()
{
    PrimaryActorTick.bCanEverTick = true;

    AircraftMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("AircraftMesh"));
    RootComponent = AircraftMesh;

    // Load a default aircraft mesh if you have one
    //static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/Game/CommercialPlane/Meshes/Collision/SM_col_CommercialPlane"));
    //if (MeshAsset.Succeeded())
    //    AircraftMesh->SetStaticMesh(MeshAsset.Object);
}

void AAircraftActor::BeginPlay()
{
    Super::BeginPlay();
}

void AAircraftActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void AAircraftActor::UpdateState(const FVector& Position, const FVector& Velocity, const FRotator& Rotation, const FVector& AngularVelocity)
{
    SetActorLocation(Position * 100.0f); // Convert meters to centimeters for Unreal
    SetActorRotation(Rotation);

    // You can use Velocity and AngularVelocity for additional effects or physics if needed
}