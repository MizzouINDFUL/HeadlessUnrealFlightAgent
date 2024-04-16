// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MoviePipelineImageSequenceOutput.h"
#include "MoviePipelineROSOutput.generated.h"

/**
 * 
 */
UCLASS()
class MOVIERENDERQUEUETOROS_API UMoviePipelineROSOutput : public UMoviePipelineImageSequenceOutputBase
{
	GENERATED_BODY()
public:
#if WITH_EDITOR
	virtual FText GetDisplayText() const override { return NSLOCTEXT("MovieRenderPipeline", "ImgSequenceROSSettingDisplayName", "ROS Message Sequence [8bit]"); }
#endif

public:
	UMoviePipelineROSOutput()
	{
		
	};

	virtual void OnReceiveImageDataImpl(FMoviePipelineMergerOutputFrame* InMergedOutputFrame) override;
	virtual bool IsAlphaAllowed() const override { return false; }

private:
	void RGBtoBGR(uint8* Data, int32 Width, int32 Height);
};
