// Fill out your copyright notice in the Description page of Project Settings.


#include "VisionLib.h"
#include "VisionActor.h"
#include "Kismet/GameplayStatics.h"


void UVisionLib::AddViewportCapture(UObject* WorldContext){
    //find all actors with tag "ViewportCapture". If one exists, dont create a new one
    TArray<AActor*> ViewportCaptureActors;
    UGameplayStatics::GetAllActorsWithTag(WorldContext, FName("ViewportCapture"), ViewportCaptureActors);
    if(ViewportCaptureActors.Num() > 0){
        UE_LOG(LogTemp, Warning, TEXT("ViewportCapture already exists"));
        return;
    }

    //create new Blueprint class. Reference: /Script/Engine.Blueprint'/ROSIntegrationVision/BP_CinematicViewportCapture.BP_CinematicViewportCapture'
    UClass* BPClass = LoadClass<AActor>(NULL, TEXT("/ROSIntegrationVision/BP_CinematicViewportCapture.BP_CinematicViewportCapture_C"));
    if(!BPClass){
        UE_LOG(LogTemp, Warning, TEXT("Could not load BP_CinematicViewportCapture"));
        return;
    }

    //spawn new actor
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AActor* NewActor = WorldContext->GetWorld()->SpawnActor<AActor>(BPClass, SpawnParams);

    UE_LOG(LogTemp, Warning, TEXT("ViewportCapture created")); 
}

void UVisionLib::AddVisionActor(UObject* WorldContext, FString TagsToTrack){

    //The format of the input String is going to be: "Tag, Tag, ..."
    //1. Split the string at the commas
    TArray<FString> Tags;
    TagsToTrack.ParseIntoArray(Tags, TEXT(","), true);

    //2. Convert to FNames
    TArray<FName> TagNames;
    for(FString Tag : Tags){
        TagNames.AddUnique(FName(*Tag));
    }
    
    //Find all AVisionActor actors
    TArray<AActor*> VisionActors;
    UGameplayStatics::GetAllActorsOfClass(WorldContext, AVisionActor::StaticClass(), VisionActors);
    if(VisionActors.Num() > 0){
        if (AVisionActor* VisionActor = Cast<AVisionActor>(VisionActors[0])){
            VisionActor->GetVisionComponent()->SetTagsToTrack(TagNames);
        }
        UE_LOG(LogTemp, Warning, TEXT("VisionActor already exists"));
        return;
    }

    //create a new VisionActor
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AVisionActor* NewActor = WorldContext->GetWorld()->SpawnActor<AVisionActor>(SpawnParams);
    NewActor->GetVisionComponent()->SetTagsToTrack(TagNames);
    
    UE_LOG(LogTemp, Warning, TEXT("VisionActor created"));
}

void UVisionLib::TagActorsFromString(UObject* WorldContext, FString InStr)
{
    //The format of the input String is going to be: "nameOfActor:Tag, NameofActor:Tag, ..."
    //1. Split the string at the commas
    TArray<FString> ActorTagPairs;
    InStr.ParseIntoArray(ActorTagPairs, TEXT(","), true);

    //2. Split each pair at the colon
    for(FString ActorTagPair : ActorTagPairs){
        TArray<FString> ActorTagPairArray;
        ActorTagPair.ParseIntoArray(ActorTagPairArray, TEXT(":"), true);

        //3. Get the actor with the name
        TArray<AActor*> Actors;
        UGameplayStatics::GetAllActorsOfClass(WorldContext, AActor::StaticClass(), Actors);
        for(AActor* Actor : Actors){
            if(Actor->GetName() == ActorTagPairArray[0]){
                //4. Set the tag
                Actor->Tags.AddUnique(FName(*ActorTagPairArray[1]));
                UE_LOG(LogTemp, Warning, TEXT("Tagged %s with %s"), *ActorTagPairArray[0], *ActorTagPairArray[1]);
            }
        }
    }
}
