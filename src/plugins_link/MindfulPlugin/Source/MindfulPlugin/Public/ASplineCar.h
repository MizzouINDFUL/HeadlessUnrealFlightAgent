#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SplineComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "ASplineCar.generated.h"

UCLASS()
class MINDFULPLUGIN_API AASplineCar : public AActor
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    AASplineCar();

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;

private:
    UPROPERTY(VisibleAnywhere)
    UStaticMeshComponent* CarMesh;

    UPROPERTY(VisibleAnywhere)
    USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, meta = (AllowPrivateAccess = "true"))
    USplineComponent* CarPath;

    UPROPERTY(EditAnywhere)
    bool bLoop = true;

    UPROPERTY(EditAnywhere)
    float MaxSpeed = 100.0f;

    UPROPERTY(EditAnywhere)
    float Acceleration = 10.0f;
    
};
