#include "ASplineCar.h"
#include "ATrackerActor.h"
#include "UObject/ConstructorHelpers.h"

// Include any additional headers here

AASplineCar::AASplineCar()
{
    // Set this actor to call Tick() every frame
    PrimaryActorTick.bCanEverTick = true;

    // Set up the root component
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
    SetRootComponent(RootComponent);

    //Set Actor's mobility to Moveable
    RootComponent->SetMobility(EComponentMobility::Movable);

    // Initialize the Spline Component
    CarPath = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));
    CarPath->SetupAttachment(RootComponent);

    // Initialize the Static Mesh Component
    CarMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMeshComponent"));
    CarMesh->SetupAttachment(RootComponent);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArmComponent"));
    SpringArm->SetupAttachment(CarMesh);

    // Set up the Spring Arm Component
    SpringArm->SetRelativeRotation(FRotator(-30.0f, 0.0f, 0.0f));
    SpringArm->TargetArmLength = 400.0f;
    SpringArm->bDoCollisionTest = false;

    // Load and assign the Static Mesh for the Static Mesh Component
    static ConstructorHelpers::FObjectFinder<UStaticMesh> StaticMeshAsset(TEXT("/Script/Engine.StaticMesh'/MindfulPlugin/CitySampleVehicles/vehicle03_Car/Mesh/SM_vehCar_vehicle03_LOD.SM_vehCar_vehicle03_LOD'"));
    if (StaticMeshAsset.Succeeded())
    {
        CarMesh->SetStaticMesh(StaticMeshAsset.Object);
    }
}

void AASplineCar::BeginPlay()
{
    Super::BeginPlay();

    // Spawn an empty AActor
    AActor* EmptyActor = GetWorld()->SpawnActor<ATrackerActor>();
    
    //add a tag to actor
    EmptyActor->Tags.Add("CarTrackActor");

    // Attach the EmptyActor to the CarMesh component
    EmptyActor->AttachToComponent(SpringArm, FAttachmentTransformRules::SnapToTargetIncludingScale);
    EmptyActor->SetActorRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
    EmptyActor->SetActorRelativeRotation(FRotator(0.0f, 0.0f, 0.0f));
}


void AASplineCar::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Move the CarMesh along the CarPath
    if (CarPath && CarMesh && CarMesh->GetStaticMesh())
    {
        // Calculate the distance to move based on MaxSpeed and Acceleration
        float DistanceToMove = MaxSpeed * DeltaTime + 0.5f * Acceleration * DeltaTime * DeltaTime;

        // Get the current location on the spline
        float CurrentDistance = CarPath->FindInputKeyClosestToWorldLocation(CarMesh->GetComponentLocation());
        CurrentDistance = CarPath->GetDistanceAlongSplineAtSplineInputKey(CurrentDistance);

        // Calculate the new distance based on bLoop
        float NewDistance = CurrentDistance + DistanceToMove;
        if (bLoop)
        {
            NewDistance = FMath::Fmod(NewDistance, CarPath->GetSplineLength());
        }
        else
        {
            NewDistance = FMath::Clamp(NewDistance, 0.f, CarPath->GetSplineLength());
        }

        // Set the new location on the spline
        FVector NewLocation = CarPath->GetLocationAtDistanceAlongSpline(NewDistance, ESplineCoordinateSpace::World);
        CarMesh->SetWorldLocation(NewLocation);
    }
}