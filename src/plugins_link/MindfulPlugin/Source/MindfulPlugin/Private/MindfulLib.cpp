// Fill out your copyright notice in the Description page of Project Settings.


#include "MindfulLib.h"
#include "Kismet/GameplayStatics.h"
#include "RainFollow.h"

#include "Settings/LevelEditorPlaySettings.h"
#include "Editor/UnrealEdEngine.h"
#include "Editor/EditorEngine.h"
#include "UnrealEdGlobals.h"
#include "Blueprint/WidgetLayoutLibrary.h"
#include "Editor/UnrealEd/Public/Editor.h"

#include "MindfulNotifier.h"

void UMindfulLib::AddNotifier(UObject* WorldContext)
{
    //Check if there is already an instance of the actor
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(WorldContext, AMindfulNotifier::StaticClass(), FoundActors);
    if (FoundActors.Num() > 0)
    {
        return;
    }

    //Spawn the AMindfulNotifier actor
    UWorld* World = WorldContext->GetWorld(); //GEngine->GetWorldFromContextObject(WorldContext);
    FActorSpawnParameters SpawnParams;
    World->SpawnActor<AMindfulNotifier>(SpawnParams);
}

void UMindfulLib::AddRainFollow(UObject* WorldContext, float Intensity)
{
    //Check if there is already an instance of the actor
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(WorldContext, ARainFollow::StaticClass(), FoundActors);
    if (FoundActors.Num() > 0)
    {
        ARainFollow* RainFollow = Cast<ARainFollow>(FoundActors[0]);
        if (RainFollow)
        {
            RainFollow->SetRainIntensity(Intensity);
        }
        return;
    }

    //Spawn the ARainFollow actor
    UWorld* World = WorldContext->GetWorld(); //GEngine->GetWorldFromContextObject(WorldContext);
    FActorSpawnParameters SpawnParams;
    ARainFollow* Rain = World->SpawnActor<ARainFollow>(SpawnParams);
    Rain->SetRainIntensity(Intensity);
}

//Stop the Unreal Play In Editor session
void UMindfulLib::StopLife(){
    GUnrealEd->RequestEndPlayMap();
}

void UMindfulLib::StartLife(){
    FRequestPlaySessionParams sessionParameters;

    GUnrealEd->RequestPlaySession(sessionParameters);
}

FBox2D UMindfulLib::GetAABB(AActor* forActor){
    FBox2D Box2D(ForceInit);

    if (forActor)
    {
        APlayerController* PlayerController = UGameplayStatics::GetPlayerController(forActor, 0);

        if (PlayerController)
        {
            TArray<FVector> Points;
            FVector FwdVec = forActor->GetActorForwardVector();
            FVector RightVec = forActor->GetActorRightVector();
            FVector UpVec = forActor->GetActorUpVector();
            FVector Loc = forActor->GetActorLocation();
            FVector Extent = forActor->GetComponentsBoundingBox().GetExtent();

            Points.Add(Loc + FwdVec * Extent.X + RightVec * Extent.Y + UpVec * Extent.Z);
            Points.Add(Loc + FwdVec * Extent.X + RightVec * Extent.Y - UpVec * Extent.Z);
            Points.Add(Loc + FwdVec * Extent.X - RightVec * Extent.Y + UpVec * Extent.Z);
            Points.Add(Loc + FwdVec * Extent.X - RightVec * Extent.Y - UpVec * Extent.Z);
            Points.Add(Loc - FwdVec * Extent.X + RightVec * Extent.Y + UpVec * Extent.Z);
            Points.Add(Loc - FwdVec * Extent.X + RightVec * Extent.Y - UpVec * Extent.Z);
            Points.Add(Loc - FwdVec * Extent.X - RightVec * Extent.Y + UpVec * Extent.Z);
            Points.Add(Loc - FwdVec * Extent.X - RightVec * Extent.Y - UpVec * Extent.Z);

            TArray<FVector2D> Points2D;

            for (int32 i = 0; i < Points.Num(); i++)
            {
                FVector2D Point2D;
                UWidgetLayoutLibrary::ProjectWorldLocationToWidgetPosition(PlayerController, Points[i], Point2D, false);
                Points2D.Add(Point2D);
            }

            float MinX = Points2D[0].X;
            float MinY = Points2D[0].Y;
            float MaxX = Points2D[0].X;
            float MaxY = Points2D[0].Y;

            for(int32 i = 1; i < Points2D.Num(); i++)
            {
                if (Points2D[i].X < MinX)
                {
                    MinX = Points2D[i].X;
                }
                if (Points2D[i].Y < MinY)
                {
                    MinY = Points2D[i].Y;
                }
                if (Points2D[i].X > MaxX)
                {
                    MaxX = Points2D[i].X;
                }
                if (Points2D[i].Y > MaxY)
                {
                    MaxY = Points2D[i].Y;
                }
            }

            return FBox2D(FVector2D(MinX, MinY), FVector2D(MaxX, MaxY));
        }
    }

    return Box2D;
}

