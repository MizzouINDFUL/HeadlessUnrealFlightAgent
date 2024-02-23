#include "RainFollow.h"
#include "Kismet/GameplayStatics.h"
#include "NiagaraSystem.h"

ARainFollow::ARainFollow()
{
    RainVFX = CreateDefaultSubobject<UNiagaraComponent>(TEXT("RainVFX"));
    USceneComponent* SceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("SceneComponent"));
    SetRootComponent(SceneComponent);
    // Set default values for RainVFX
    static ConstructorHelpers::FObjectFinder<UNiagaraSystem> RainVFXAsset(TEXT("/MindfulPlugin/VFX/VFX_Rain.VFX_Rain"));
    if (RainVFXAsset.Succeeded())
    {
        RainVFX->SetAsset(RainVFXAsset.Object);
    }
    
    // RainVFX->SetupAttachment(RootComponent);
    RainVFX->SetRelativeLocation(FVector(0.f, 0.f, 1000.f));

    RainVFX->SetAutoActivate(false);
    Tags.Add(FName("Rain"));
}

void ARainFollow::BeginPlay()
{
    Super::BeginPlay();
    
    SetActorLocation(FVector(0.f, 0.f, 10000.f));
}

void ARainFollow::SetRainIntensity(float Intensity)
{
    // Set the Niagara particle system's intensity
    RainVFX->SetFloatParameter("RainIntensity", Intensity);
    // Start the Niagara particle system
    RainVFX->Activate();
}

void ARainFollow::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
    // auto PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
    // if (PlayerPawn)
    // {
    //     // Get the player's camera location
    //     FVector CameraLocation = PlayerPawn->GetActorLocation();
        
    //     // Calculate the target location with the offset
    //     FVector TargetLocation = CameraLocation + FVector(0, 0, 900);

    //     UE_LOG(LogTemp, Warning, TEXT("CameraLocation: %s"), *CameraLocation.ToString());
        
    //     // Set the actor's location to the target location
    //     SetActorLocation(TargetLocation);
    // }

    // Update the Niagara particle system
    // Add any additional logic here
}
