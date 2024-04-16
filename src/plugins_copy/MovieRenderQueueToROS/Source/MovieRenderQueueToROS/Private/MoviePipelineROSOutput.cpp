// Fill out your copyright notice in the Description page of Project Settings.


#include "MoviePipelineROSOutput.h"
#include "sensor_msgs/Image.h"
#include "RI/Topic.h"
#include "Kismet/GameplayStatics.h"
#include "ROSIntegrationGameInstance.h"

void UMoviePipelineROSOutput::RGBtoBGR(uint8* Data, int32 Width, int32 Height)
{
	for (int32 y = 0; y < Height; y++)
	{
		for (int32 x = 0; x < Width; x++)
		{
			uint8* Pixel = Data + (y * Width + x) * 3;
			uint8 Temp = Pixel[0];
			Pixel[0] = Pixel[2];
			Pixel[2] = Temp;
		}
	}
}

void UMoviePipelineROSOutput::OnReceiveImageDataImpl(FMoviePipelineMergerOutputFrame* InMergedOutputFrame)
{
	Super::OnReceiveImageDataImpl(InMergedOutputFrame);

	check(InMergedOutputFrame);

	FROSTime time = FROSTime::Now();

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

	for (const FMoviePipelinePassIdentifier& Pass : Passes)
	{

		TUniquePtr<FImagePixelData>& PixelData = InMergedOutputFrame->ImageOutputData[Pass];

		const void* RawData = nullptr;
		int64 SizeBytes = 0;
		PixelData->GetRawData(RawData, SizeBytes);

		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: SizeBytes: %d"), SizeBytes);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Width: %d"), PixelData->GetSize().X);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Height: %d"), PixelData->GetSize().Y);
		UE_LOG(LogTemp, Warning, TEXT("MoviePipelineROSOutput: Width x Height x 3: %d"), PixelData->GetSize().X * PixelData->GetSize().Y * 3);

		// the required output for a ROS IMage message is an BGR8 image
		// RawData is a Float16 image, so we need to convert it to 8bit
		// we need to convert the image to BGR8

		uint8* Data = (uint8*)RawData;
		float* FloatData = (float*)RawData;

		//  convert the image to BGR8
		const int32 NumPixels = PixelData->GetSize().X * PixelData->GetSize().Y;
		for(int32 i = 0; i < NumPixels; ++i, ++Data, ++FloatData)
		{
			uint8 R = FMath::Clamp(FMath::RoundToInt(FMath::Clamp<float>(FloatData[0], 0.0f, 1.0f) * 255.0f), 0, 255);
			uint8 G = FMath::Clamp(FMath::RoundToInt(FMath::Clamp<float>(FloatData[1], 0.0f, 1.0f) * 255.0f), 0, 255);
			uint8 B = FMath::Clamp(FMath::RoundToInt(FMath::Clamp<float>(FloatData[2], 0.0f, 1.0f) * 255.0f), 0, 255);

			Data[0] = B; // BGR8 [0] = B, [1] = G, [2] = R
			Data[1] = G;
			Data[2] = R;
		}

		UTopic* ImageTopic = NewObject<UTopic>(UTopic::StaticClass());
		ImageTopic->Init(rosinst->ROSIntegrationCore, Pass.Name, "sensor_msgs/Image");
		ImageTopic->Advertise();

		TSharedPtr<ROSMessages::sensor_msgs::Image> ImageMessage(new ROSMessages::sensor_msgs::Image());
		ImageMessage->header.seq = 0;
		ImageMessage->header.time = time;
		ImageMessage->height = PixelData->GetSize().Y;
		ImageMessage->width = PixelData->GetSize().X;
		ImageMessage->encoding = "bgr8";
		ImageMessage->is_bigendian = 0;
		ImageMessage->step = PixelData->GetSize().X * 3;
		ImageMessage->data = Data;

		const bool PublishSuccess = ImageTopic->Publish(ImageMessage);
	}
}
