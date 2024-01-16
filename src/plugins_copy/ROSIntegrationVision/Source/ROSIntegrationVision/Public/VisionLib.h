// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "VisionLib.generated.h"

/**
 * 
 */
UCLASS()
class ROSINTEGRATIONVISION_API UVisionLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:

	UFUNCTION(BlueprintCallable, Category = "VisionLib")
	static void AddViewportCapture(UObject* WorldContext);

	UFUNCTION(BlueprintCallable, Category = "VisionLib")
	static void AddVisionActor(UObject* WorldContext, FString TagsToTrack);

	UFUNCTION(BlueprintCallable, Category = "VisionLib")
	static void TagActorsFromString(UObject* WorldContext, FString InStr);

};
