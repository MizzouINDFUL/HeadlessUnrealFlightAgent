#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "RainFollow.generated.h"

UCLASS()
class MINDFULPLUGIN_API ARainFollow : public AActor
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ARainFollow();

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
    class UNiagaraComponent* RainVFX;

    UFUNCTION(BlueprintCallable, Category = "Niagara")
    void SetRainIntensity(float Intensity);
};