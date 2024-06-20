#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "ROSLib.generated.h"

UCLASS()
class UROSLib : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "ROSIntegration")
    static void TestImageRead(const FString& ImagePath);

    UFUNCTION(BlueprintCallable, Category = "ROSIntegration")
    static void ReadTest(const FString& ImagePath);

    UFUNCTION(BlueprintPure, Category = "ROSIntegration")
    static FString FormatLevelSequenceFrameNumber(int32 FrameNumber);
};