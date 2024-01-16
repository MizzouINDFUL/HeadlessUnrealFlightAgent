#include "ATrackerActor.h"

// Include any additional headers you may need

ATrackerActor::ATrackerActor()
{
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
    SetRootComponent(RootComponent);
    RootComponent->SetMobility(EComponentMobility::Movable);
}


void ATrackerActor::BeginPlay()
{
    Super::BeginPlay();
    
    // Add any initialization code you need to run when the actor is spawned
}


void ATrackerActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
    // Add any code you need to run every frame
}



// Add any additional member function definitions here

