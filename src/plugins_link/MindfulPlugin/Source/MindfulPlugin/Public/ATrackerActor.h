#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ATrackerActor.generated.h"

UCLASS()
class MINDFULPLUGIN_API ATrackerActor : public AActor
{
    GENERATED_BODY()

public:
    // Constructor
    ATrackerActor();

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;
};
