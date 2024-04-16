#include "ROSLib.h"
#include "GenericPlatform/GenericPlatformProcess.h"
#include "Misc/FileHelper.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "RenderUtils.h"

#if WITH_OPENCV

#include "PreOpenCVHeaders.h"
#include <opencv2/imgcodecs.hpp>
#include "PostOpenCVHeaders.h"

#endif

//by default, movie render queue titles frames as following: 0000, 0001, 0002, ... 0021, 0022, ... 0100, 0101, ...
//this function takes in a frame number and returns a string formatted as 0000, 0001, 0002, ... 0021, 0022, ... 0100, 0101, ...
FString UROSLib::FormatLevelSequenceFrameNumber(int32 FrameNumber)
{
    FString FrameNumberString = FString::FromInt(FrameNumber);
    FString FormattedFrameNumber = FrameNumberString;
    int32 FrameNumberLength = FrameNumberString.Len();
    int32 NumberOfZeros = 4 - FrameNumberLength;
    for (int32 i = 0; i < NumberOfZeros; i++)
    {
        FormattedFrameNumber = "0" + FormattedFrameNumber;
    }
    return FormattedFrameNumber;
}

void UROSLib::TestImageRead(const FString& ImagePath)
{

    bool canBeRead = FPaths::FileExists(ImagePath);

    if (!canBeRead)
    {
        UE_LOG(LogTemp, Error, TEXT("FPaths: File does not exist"));
    }
    else{
        UE_LOG(LogTemp, Warning, TEXT("FPaths: File exists"));
    }

    TArray<uint8> ImageData;
    if(FFileHelper::LoadFileToArray(ImageData, *ImagePath))
    {
        UE_LOG(LogTemp, Warning, TEXT("Image read successfully"));

        IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));

        EImageFormat ImageFormat = ImageWrapperModule.DetectImageFormat(ImageData.GetData(), ImageData.Num());
        if (ImageFormat == EImageFormat::Invalid)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to detect image format for file: %s"), *ImagePath);
            return;
        }

        TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(ImageFormat);
        
        TArray<uint8> RawData;
        ImageWrapper->SetCompressed(ImageData.GetData(), ImageData.Num());
        ImageWrapper->GetRaw(ERGBFormat::BGRA, 8, RawData);
        if (RawData.Num() == 0)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to decompress image file: %s"), *ImagePath);
            return;
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Image decompressed successfully"));
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("FFileHelper: Could not read image"));
    }

    /*
    cv::Mat Image = cv::imread(TCHAR_TO_UTF8(*ImagePath), cv::IMREAD_UNCHANGED);

    if (Image.empty())
    {
        UE_LOG(LogTemp, Error, TEXT("OpenCV could not open or find the image at %s"), *ImagePath);
        return;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Image read successfully"));
    }
    */
}

void UROSLib::ReadTest(const FString& ImagePath)
{
    UE_LOG(LogTemp, Warning, TEXT("ReadTest called"));

    std::string path = TCHAR_TO_UTF8(*ImagePath);

    bool canBeRead = FPaths::FileExists(ImagePath);

    if (!canBeRead)
    {
        UE_LOG(LogTemp, Error, TEXT("FPaths: File does not exist"));
        return;
    }
    else{
        UE_LOG(LogTemp, Warning, TEXT("FPaths: File exists"));
    }
}