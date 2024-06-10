// Fill out your copyright notice in the Description page of Project Settings.


#include "MoviePipelineROSOutput.h"
#include <cmath>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "RI/Topic.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "MoviePipeline.h"
#include "MoviePipelineImageQuantization.h"
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
		FVector CameraLocation = CameraManager->GetCameraLocation();
		FRotator CameraRotation = CameraManager->GetCameraRotation();

		UTopic* PoseTopic = NewObject<UTopic>(UTopic::StaticClass());
		const FString PoseTopicName = "/" + SessionName + "/unreal_ros/pose";
		PoseTopic->Init(rosinst->ROSIntegrationCore, PoseTopicName, "geometry_msgs/Pose");
		PoseTopic->Advertise();

		TSharedPtr<ROSMessages::geometry_msgs::Pose> PoseMessage(new ROSMessages::geometry_msgs::Pose());
		PoseMessage->position.x = CameraLocation.X;
		PoseMessage->position.y = CameraLocation.Y;
		PoseMessage->position.z = CameraLocation.Z;
		PoseMessage->orientation = FQuat(CameraRotation).GetNormalized();

		const bool PublishPoseSuccess = PoseTopic->Publish(PoseMessage);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Published Pose to %s: %s"), *PoseTopicName, PublishPoseSuccess ? TEXT("Success") : TEXT("Failed"));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("MoviePipelineROSOutput: Could not get CameraManager"));
	}
}
