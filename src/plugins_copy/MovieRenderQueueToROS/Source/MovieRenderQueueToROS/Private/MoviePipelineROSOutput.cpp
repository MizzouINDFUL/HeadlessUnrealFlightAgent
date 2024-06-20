// Fill out your copyright notice in the Description page of Project Settings.


#include "MoviePipelineROSOutput.h"
#include <cmath>
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "RI/Topic.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "MoviePipeline.h"
#include "MoviePipelineImageQuantization.h"
#include "CineCameraActor.h"
#include "ROSIntegrationGameInstance.h"

void UMoviePipelineROSOutput::RGBAtoRGB(uint8* inRGBA, uint8* outRGB, int32 Width, int32 Height)
{
	for (int32 y = 0; y < Height; y++)
	{
		for (int32 x = 0; x < Width; x++)
		{
			int32 Index = (y * Width + x) * 4;
			outRGB[y * Width + x] = inRGBA[Index];
			outRGB[y * Width + x + 1] = inRGBA[Index + 1];
			outRGB[y * Width + x + 2] = inRGBA[Index + 2];
		}
	}
}

void UMoviePipelineROSOutput::RGBtoBGR(uint8* Data, int32 Width, int32 Height)
{
	for (int32 y = 0; y < Height; y++)
	{
		for (int32 x = 0; x < Width; x++)
		{
			int32 Index = (y * Width + x) * 3;
			uint8 Temp = Data[Index];
			Data[Index] = Data[Index + 2];
			Data[Index + 2] = Temp;
		}
	}
}

UCineCameraComponent* UMoviePipelineROSOutput::TryGetCineCameraComponent()
{
	//get player pawn
	APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(this, 0);
	if(PlayerPawn)
	{
		//Iterate through components of player pawn
		for (UActorComponent* ActorComponent : PlayerPawn->GetComponents())
		{
			UCineCameraComponent* CineCameraComponent = Cast<UCineCameraComponent>(ActorComponent);
			if(CineCameraComponent)
			{
				//check if this is an active camera
				if(CineCameraComponent->IsActive())
				{
					return CineCameraComponent;
				}
			}
		}
	}

	//find all CineCameraActors
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(this, ACineCameraActor::StaticClass(), FoundActors);

	//iterate through them and look at their CineCameraComponent. If any of those is active - return it
	for (AActor* Actor : FoundActors)
	{
		ACineCameraActor* CineCameraActor = Cast<ACineCameraActor>(Actor);
		if(CineCameraActor)
		{
			UCineCameraComponent* CineCameraComponent = CineCameraActor->GetCineCameraComponent();
			if(CineCameraComponent && CineCameraComponent->IsActive())
			{
				return CineCameraComponent;
			}
		}
	}

	return nullptr;
}

void UMoviePipelineROSOutput::OnReceiveImageDataImpl(FMoviePipelineMergerOutputFrame* InMergedOutputFrame)
{
	Super::OnReceiveImageDataImpl(InMergedOutputFrame);

	check(InMergedOutputFrame);

	FROSTime time = FROSTime::Now();

	FString SessionName = "";
	if(FParse::Value(FCommandLine::Get(), TEXT("session_name="), SessionName))
	{
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Found session_name argument: %s"), *SessionName);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Did not find session_name argument"));
	}

	float TargetHeight = 0.0f;
	float TargetWidth = 0.0f;

	//iterate over the arguments passed to UnrealEditor-Cmd.sh
	FString TopicArgs = "";
	if (FParse::Value(FCommandLine::Get(), TEXT("topic_to_layer="), TopicArgs))
	{
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Found topic_to_layer argument: %s"), *TopicArgs);

		TArray<FString> TopicToLayerPairs;
		TopicArgs.ParseIntoArray(TopicToLayerPairs, TEXT("+"), true);

		for (FString TopicToLayerPair : TopicToLayerPairs)
		{
			TArray<FString> TopicLayerPair;
			TopicToLayerPair.ParseIntoArray(TopicLayerPair, TEXT(":"), true);
			if (TopicLayerPair.Num() == 2)
			{
				RenderPassToTopicMap.Add(TopicLayerPair[1], TopicLayerPair[0]);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Invalid topic_to_layer argument: %s"), *TopicToLayerPair);
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Did not find topic_to_layer argument"));
	}

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(UGameplayStatics::GetGameInstance(this));
	if (!rosinst)
	{
		UE_LOG(LogTemp, Error, TEXT("MoviePipelineROSOutput: Could not get ROSIntegrationGameInstance"));
		return;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Got ROSIntegrationGameInstance"));
	}

	TArray<FMoviePipelinePassIdentifier> Passes;
	InMergedOutputFrame->ImageOutputData.GetKeys(Passes);

	//tmp: list the size of RenderPassToTopicMap
	UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: RenderPassToTopicMap size: %d"), RenderPassToTopicMap.Num());

	// tmp: list all keys and calues in 
	for (auto& Elem : RenderPassToTopicMap)
	{
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: RenderPassToTopicMap: %s -> %s"), *Elem.Key, *Elem.Value);
	}

	for (const FMoviePipelinePassIdentifier& Pass : Passes)
	{

		const FString TopicName = RenderPassToTopicMap.Contains(Pass.Name) ? RenderPassToTopicMap[Pass.Name] : Pass.Name;

		//Log Pass Name
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Pass Name: %s"), *Pass.Name);
		

		TUniquePtr<FImagePixelData>& PixelData = InMergedOutputFrame->ImageOutputData[Pass];
		TUniquePtr<FImagePixelData> QuantizedPixelData = UE::MoviePipeline::QuantizeImagePixelDataToBitDepth(PixelData.Get(), 8, nullptr, true);
		const void* RawData = nullptr;
		int64 SizeBytes = 0;
		QuantizedPixelData->GetRawData(RawData, SizeBytes);

		if(TargetWidth == 0) TargetWidth = PixelData->GetSize().X;
		if(TargetHeight == 0) TargetHeight = PixelData->GetSize().Y;

		uint8* BGR = new uint8[PixelData->GetSize().X * PixelData->GetSize().Y * 3];
		const FColor *itI = (FColor*)RawData;
		uint8_t *itO = BGR;

		// Converts Float colors to bytes
		for (size_t i = 0; i < PixelData->GetSize().X * PixelData->GetSize().Y; ++i, ++itI, ++itO)
		{
			*itO = (uint8_t)std::round(FMath::Clamp((float)itI->B, 0.f, 255.f));
			*++itO = (uint8_t)std::round(FMath::Clamp((float)itI->G, 0.f, 255.f));
			*++itO = (uint8_t)std::round(FMath::Clamp((float)itI->R, 0.f, 255.f));
		}

		UTopic* ImageTopic = NewObject<UTopic>(UTopic::StaticClass());
		const FString FinalTopicName = "/" + SessionName + TopicName;
		ImageTopic->Init(rosinst->ROSIntegrationCore, FinalTopicName, "sensor_msgs/Image");
		ImageTopic->Advertise();

		TSharedPtr<ROSMessages::sensor_msgs::Image> ImageMessage(new ROSMessages::sensor_msgs::Image());
		ImageMessage->header.seq = 0;
		ImageMessage->header.time = time;
		ImageMessage->height = PixelData->GetSize().Y;
		ImageMessage->width = PixelData->GetSize().X;
		ImageMessage->encoding = TEXT("bgr8");
		ImageMessage->is_bigendian = 0;
		ImageMessage->step = PixelData->GetSize().X * 3;
		ImageMessage->data = BGR;

		const bool PublishSuccess = ImageTopic->Publish(ImageMessage);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Published Image to %s: %s"), *FinalTopicName, PublishSuccess ? TEXT("Success") : TEXT("Failed"));
	}

	//get player camera location and rotation to export Pose
	APlayerCameraManager* CameraManager = UGameplayStatics::GetPlayerCameraManager(this, 0);
	if(CameraManager)
	{
		//Current Camera Transform
		FVector CameraLocation = CameraManager->GetCameraLocation();
		FRotator CameraRotation = CameraManager->GetCameraRotation();

		FVector FwdVector = UKismetMathLibrary::GetForwardVector(CameraRotation);
		FVector UpVector = UKismetMathLibrary::GetUpVector(CameraRotation);

		UTopic* StringTopic = NewObject<UTopic>(UTopic::StaticClass());
		const FString StringTopicName = "/" + SessionName + "/unreal_ros/camera_position";
		StringTopic->Init(rosinst->ROSIntegrationCore, StringTopicName, "std_msgs/String");
		StringTopic->Advertise();

		FString JsonString = FString::Printf(TEXT("{\"annotations\": [ { \"CamVecStraight_XYZ\": [ %.3f, %.3f, %.3f ], \"CamVecUp_XYZ\": [ %.3f, %.3f, %.3f ], \"cPitch\": %.3f, \"cRoll\": %.3f, \"cX\": %.3f, \"cY\": %.3f, \"cYaw\": %.3f, \"cZ\": %.3f } ] }"),
			FwdVector.X, FwdVector.Y, FwdVector.Z,
			UpVector.X, UpVector.Y, UpVector.Z,
			CameraRotation.Pitch, CameraRotation.Roll,
			CameraLocation.X, CameraLocation.Y, CameraRotation.Yaw, CameraLocation.Z);

		TSharedPtr<ROSMessages::std_msgs::String> StringMessage(new ROSMessages::std_msgs::String(JsonString));

		const bool PublishStringSuccess = StringTopic->Publish(StringMessage);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Published Camera Transform String to %s: %s"), *StringTopicName, PublishStringSuccess ? TEXT("Success") : TEXT("Failed"));
	
		//General Camera Info
		//only do it if player is using a cineCameraComponent
		UCineCameraComponent* CineCameraComponent = TryGetCineCameraComponent();
		if (!CineCameraComponent)
		{
			UE_LOG(LogTemp, Error, TEXT("MoviePipelineROSOutput: Could not get CineCameraComponent"));
			return;
		}

		UTopic* CameraInfoTopic = NewObject<UTopic>(UTopic::StaticClass());
		const FString CameraInfoTopicName = "/" + SessionName + "/unreal_ros/camera_info";
		CameraInfoTopic->Init(rosinst->ROSIntegrationCore, CameraInfoTopicName, "std_msgs/String");
		CameraInfoTopic->Advertise();

		float FOV = CameraManager->GetFOVAngle();
		float FocusDistance = CineCameraComponent->CurrentFocusDistance;
		float FocalLength = CineCameraComponent->CurrentFocalLength;
		float ImageH = TargetHeight;
		float ImageW = TargetWidth;
		float PixelAspectRatio = ImageW / ImageH;
		float SensorHeight = CineCameraComponent->Filmback.SensorHeight;
		float SensorWidth = CineCameraComponent->Filmback.SensorWidth;
		float SensorAspectRatio = CineCameraComponent->Filmback.SensorAspectRatio;

		FString CameraInfoJsonString = FString::Printf(TEXT("{\"FOV\": %.6f, \"FocalDistance\": %.1f, \"FocalLength\": %.6f, \"ImageHeight\": %.1f, \"ImageWidth\": %.1f, \"PixelAspectRatio\": %.1f, \"SensorAspectRatio\": %.6f, \"SensorHeight\": %.6f, \"SensorWidth\": %.6f, \"Spectrum\": \"RGB\", \"Version\": \"Full_Sim_V1\"}"),
			FOV, FocusDistance, FocalLength, ImageH, ImageW, PixelAspectRatio, SensorAspectRatio, SensorHeight, SensorWidth);

		TSharedPtr<ROSMessages::std_msgs::String> CameraInfoMessage(new ROSMessages::std_msgs::String(CameraInfoJsonString));

		const bool PublishCameraInfoSuccess = CameraInfoTopic->Publish(CameraInfoMessage);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Published Camera Info String to %s: %s"), *CameraInfoTopicName, PublishCameraInfoSuccess ? TEXT("Success") : TEXT("Failed"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("MoviePipelineROSOutput: Could not get CameraManager"));
	}
}
