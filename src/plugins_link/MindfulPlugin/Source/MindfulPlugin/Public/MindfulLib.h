// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MindfulLib.generated.h"

/**
 * 
 */
UCLASS()
class MINDFULPLUGIN_API UMindfulLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:

	UFUNCTION(BlueprintCallable, Category = "MindfulLib")
	static void AddNotifier(UObject* WorldContext);

	UFUNCTION(BlueprintCallable, Category="MindfulLib")
	static void StartLife();

	UFUNCTION(BlueprintCallable, Category="MindfulLib")
	static void StopLife();

	UFUNCTION(BlueprintCallable, Category="MindfulLib")
	static FBox2D GetAABB(AActor* forActor);

};
