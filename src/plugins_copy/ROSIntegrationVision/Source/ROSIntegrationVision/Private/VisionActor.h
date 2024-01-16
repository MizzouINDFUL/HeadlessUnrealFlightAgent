// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"

#include "VisionComponent.h"

#include "VisionActor.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API AVisionActor : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AVisionActor();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	UFUNCTION()
	FORCEINLINE UVisionComponent * GetVisionComponent() const { return vision; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vision Actor")
		UVisionComponent * vision; 
};
