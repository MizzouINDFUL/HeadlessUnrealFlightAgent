// Fill out your copyright notice in the Description page of Project Settings.

#include "VisionActor.h"
#include "Kismet/GameplayStatics.h"
#include "ROSIntegrationGameInstance.h"

// Sets default values
AVisionActor::AVisionActor() : AActor()
{
	UE_LOG(LogTemp, Warning, TEXT("VisionActor CTOR"));

	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComponent);
    
    vision = CreateDefaultSubobject<UVisionComponent>(TEXT("Vision"));
    vision->ParentLink = "/world";
    vision->SetupAttachment(RootComponent);
}

// Called when the game starts or when spawned
void AVisionActor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AVisionActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    //set thi actor location and rotation to Player Cmaera Manager location and rotation
    if(UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0)){
        SetActorLocation(UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0)->GetCameraLocation());
        SetActorRotation(UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0)->GetCameraRotation());
    }
}