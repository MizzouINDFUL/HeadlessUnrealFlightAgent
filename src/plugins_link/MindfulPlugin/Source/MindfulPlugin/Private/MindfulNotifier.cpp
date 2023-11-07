// Fill out your copyright notice in the Description page of Project Settings.


#include "MindfulNotifier.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Misc/FileHelper.h"
#include "TimerManager.h"
#include "GameFramework/PlayerController.h"


// Sets default values
AMindfulNotifier::AMindfulNotifier()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AMindfulNotifier::BeginPlay()
{
	Super::BeginPlay();

	//set a json value "begin_play" to 1
	//path to json file: /shared/unreal.json
	FString Path = "/shared/unreal.json";
	FString JsonString;
	FFileHelper::LoadFileToString(JsonString, *Path);
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);
	FJsonSerializer::Deserialize(Reader, JsonObject);
	JsonObject->SetNumberField("begin_play", 1);
	FString OutputString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);
	FFileHelper::SaveStringToFile(OutputString, *Path);


	//delay the notification by 1 second
	FTimerHandle UnusedHandle;
	GetWorldTimerManager().SetTimer(UnusedHandle, this, &AMindfulNotifier::Delayed_BeginPlay, 1.0f, false);
}

void AMindfulNotifier::Delayed_BeginPlay()
{
	// //Get the reference to player contorller 
	APlayerController* PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
	FString Final = "py notify_begin_play.py";
	PlayerController->ConsoleCommand(Final, true);
}

void AMindfulNotifier::EndPlay(const EEndPlayReason::Type EndPlayReason)
{

	FString Path = "/shared/unreal.json";
	FString JsonString;
	FFileHelper::LoadFileToString(JsonString, *Path);
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);
	FJsonSerializer::Deserialize(Reader, JsonObject);
	JsonObject->SetNumberField("end_play", 1);
	FString OutputString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);
	FFileHelper::SaveStringToFile(OutputString, *Path);

	Super::EndPlay(EndPlayReason);
}

// Called every frame
void AMindfulNotifier::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

