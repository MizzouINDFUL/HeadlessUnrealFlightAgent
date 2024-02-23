// Author Tim Fronsee <tfronsee21@gmail.com>
#include "VisionComponent.h"

#include <cmath>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <thread>
#include "immintrin.h"

#include "ROSTime.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"

#include "PacketBuffer.h"
#include "ROSIntegrationGameInstance.h"
#include "StopTime.h"

#include "EngineUtils.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"

#if PLATFORM_WINDOWS
  #define _USE_MATH_DEFINES
#endif

// Private data container so that internal structures are not visible to the outside
class ROSINTEGRATIONVISION_API UVisionComponent::PrivateData
{
public:
	TSharedPtr<PacketBuffer> Buffer;
	// TCPServer Server;
	std::mutex WaitColor, WaitDepth, WaitObject, WaitGT, WaitDone;
	std::condition_variable CVColor, CVDepth, CVObject, CVGT, CVDone;
	std::thread ThreadColor, ThreadDepth, ThreadObject, ThreadGT;
	bool DoColor, DoDepth, DoObject, DoGT;
	bool DoneColor, DoneObject, DoneGT;
};

UVisionComponent::UVisionComponent() :
Width(1920),
Height(1080),
Framerate(100),
UseEngineFramerate(false),
ServerPort(10000),
FrameTime(1.0f / Framerate),
TimePassed(0),
ColorsUsed(0)
{
    Priv = new PrivateData();
    FieldOfView = 90.0;
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = true;
	FrameNumber = 0;
	DisableTFPublishing = true;

	SetTickGroup(TG_PostPhysics);

    auto owner = GetOwner();
    if (owner)
    {
        Color = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("ColorCapture"));
        Color->SetupAttachment(this);
        Color->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        Color->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("ColorTarget"));
        Color->TextureTarget->InitAutoFormat(Width, Height);
        Color->FOVAngle = FieldOfView;

        Depth = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DepthCapture"));
        Depth->SetupAttachment(this);
        Depth->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        Depth->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("DepthTarget"));
        Depth->TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
        Depth->TextureTarget->InitAutoFormat(Width, Height);
        Depth->FOVAngle = FieldOfView;

        Object = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("ObjectCapture"));
        Object->SetupAttachment(this);
        Object->CaptureSource = ESceneCaptureSource::SCS_SceneColorHDR;
		Object->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
		Object->bCaptureEveryFrame = false;
		Object->bCaptureOnMovement = false;
        Object->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("ObjectTarget"));
        Object->TextureTarget->InitAutoFormat(Width, Height);
        Object->FOVAngle = FieldOfView;
		Object->SetAbsolute(true, true, true);
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("No owner!"));
    }

	//try to load up a custom render target for color output. Reference: /Script/Engine.TextureRenderTarget2D'/ROSIntegrationVision/RT_ViewportCineCapture.RT_ViewportCineCapture'
	ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D> CustomColorRenderTargetFinder(TEXT("/ROSIntegrationVision/RT_ViewportCineCapture.RT_ViewportCineCapture"));
	if (CustomColorRenderTargetFinder.Succeeded()) {
		CustomColorRenderTarget = CustomColorRenderTargetFinder.Object;
	}

    // CameraInfoPublisher = NewObject<UTopic>(UTopic::StaticClass());
    DepthPublisher = NewObject<UTopic>(UTopic::StaticClass());
    ImagePublisher = NewObject<UTopic>(UTopic::StaticClass());
    // TFPublisher = NewObject<UTopic>(UTopic::StaticClass());
	ObjectsPublisher = NewObject<UTopic>(UTopic::StaticClass());
	GroundTruthPublisher = NewObject<UTopic>(UTopic::StaticClass());
	MsgCountPublisher = NewObject<UTopic>(UTopic::StaticClass());
}

UVisionComponent::~UVisionComponent()
{
    delete Priv;
}

void UVisionComponent::SetFramerate(const float _Framerate)
{
    Framerate = _Framerate;
    FrameTime = 1.0f / _Framerate;
    TimePassed = 0;
}

void UVisionComponent::Pause(const bool _Pause)
{
    Paused = _Pause;
}

bool UVisionComponent::IsPaused() const
{
    return Paused;
}

void UVisionComponent::InitializeComponent()
{
    Super::InitializeComponent();
}

void UVisionComponent::BeginPlay()
{
  Super::BeginPlay();

	//list tags we are tracking via UE_LOG
	for(int32 tagIdx = 0; tagIdx < TagsToTrack.Num(); tagIdx++)
	{
		FString thisTag = TagsToTrack[tagIdx].ToString();
		UE_LOG(LogTemp, Warning, TEXT("Tracking tag: %s"), *thisTag);
	}

    // Initializing buffers for reading images from the GPU
	ImageColor.AddUninitialized(Width * Height);
	ImageDepth.AddUninitialized(Width * Height);
	ImageObject.AddUninitialized(Width * Height);

	// Reinit renderer
	Color->TextureTarget->InitAutoFormat(Width, Height);
	Depth->TextureTarget->InitAutoFormat(Width, Height);
	Object->TextureTarget->InitAutoFormat(Width, Height);

	AspectRatio = Width / (float)Height;

	// Setting flags for each camera
	ShowFlagsLit(Color->ShowFlags);
	ShowFlagsVertexColor(Object->ShowFlags);

	// Creating double buffer and setting the pointer of the server object
	Priv->Buffer = TSharedPtr<PacketBuffer>(new PacketBuffer(Width, Height, FieldOfView));

	Running = true;
	Paused = false;

	Priv->DoColor = false;
	Priv->DoObject = false;
	Priv->DoDepth = false;
	Priv->DoGT = false;

	Priv->DoneColor = false;
	Priv->DoneObject = false;
	Priv->DoneGT = false;

	// Starting threads to process image data
	Priv->ThreadColor = std::thread(&UVisionComponent::ProcessColor, this);
	Priv->ThreadDepth = std::thread(&UVisionComponent::ProcessDepth, this);
	Priv->ThreadObject = std::thread(&UVisionComponent::ProcessObject, this);
	Priv->ThreadGT = std::thread(&UVisionComponent::ProcessGT, this);

	// Establish ROS communication
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetOwner()->GetGameInstance());
	if (rosinst)
	{
		// TFPublisher->Init(rosinst->ROSIntegrationCore,
        //               TEXT("/tf"),
        //               TEXT("tf2_msgs/TFMessage"));

		// CameraInfoPublisher->Init(rosinst->ROSIntegrationCore,
        //                       TEXT("/unreal_ros/camera_info"),
        //                       TEXT("sensor_msgs/CameraInfo"));
		// CameraInfoPublisher->Advertise();

		ImagePublisher->Init(rosinst->ROSIntegrationCore,
                         TEXT("/unreal_ros/image_color"),
                         TEXT("sensor_msgs/Image"));
		ImagePublisher->Advertise();

		DepthPublisher->Init(rosinst->ROSIntegrationCore,
                         TEXT("/unreal_ros/image_depth"),
                         TEXT("sensor_msgs/Image"));
		DepthPublisher->Advertise();

		ObjectsPublisher->Init(rosinst->ROSIntegrationCore,
						 TEXT("/unreal_ros/image_objects"),
						 TEXT("sensor_msgs/Image"));
		ObjectsPublisher->Advertise();

		MsgCountPublisher->Init(rosinst->ROSIntegrationCore,
						 TEXT("/unreal_ros/message_count"),
						 TEXT("std_msgs/String"));
		MsgCountPublisher->Advertise();

		GroundTruthPublisher->Init(rosinst->ROSIntegrationCore,
						 TEXT("/unreal_ros/ground_truth"),
						 TEXT("std_msgs/String"));
		GroundTruthPublisher->Advertise();
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("UnrealROSInstance not existing."));
	}
	SetFramerate(Framerate); // Update framerate
}

//Compare the newly captured image with the original one to find the bounding box of the actor. Pixels where we see the difference should be added to Points, Points will then construct a 2d bounding box.
FBox2D UVisionComponent::GetActorBoxFromRT(TArray<FColor> OriginalSurface, UTextureRenderTarget2D *RenderTarget) const
{
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	FReadSurfaceDataFlags ReadSurfaceDataFlags;
	ReadSurfaceDataFlags.SetLinearToGamma(false);
	ReadSurfaceDataFlags.SetOutputStencil(false);
	ReadSurfaceDataFlags.SetMip(0);
	// ReadSurfaceDataFlags.SetCubemapFace(-1);

	TArray<FColor> SurfaceData;
	RenderTargetResource->ReadPixels(SurfaceData, ReadSurfaceDataFlags);

	TArray<FVector2D> Points;

	for (int32 y = 0; y < RenderTarget->SizeY; ++y)
	{
		for (int32 x = 0; x < RenderTarget->SizeX; ++x)
		{
			if (SurfaceData[y * RenderTarget->SizeX + x] != OriginalSurface[y * RenderTarget->SizeX + x])
			{
				Points.Add(FVector2D(x, y));
			}
		}
	}

	return FBox2D(Points);
}

void UVisionComponent::DrawBox2D(UTextureRenderTarget2D *RenderTarget, FBox2D Box, FColor BoxColor) const
{
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	FReadSurfaceDataFlags ReadSurfaceDataFlags;
	ReadSurfaceDataFlags.SetLinearToGamma(false);
	ReadSurfaceDataFlags.SetOutputStencil(false);
	ReadSurfaceDataFlags.SetMip(0);
	// ReadSurfaceDataFlags.SetCubemapFace(-1);

	TArray<FColor> SurfaceData;
	RenderTargetResource->ReadPixels(SurfaceData, ReadSurfaceDataFlags);

	for (int32 y = Box.Min.Y; y < Box.Max.Y; ++y)
	{
		for (int32 x = Box.Min.X; x < Box.Max.X; ++x)
		{
			SurfaceData[y * RenderTarget->SizeX + x] = BoxColor;
		}
	}

	// RenderTargetResource->WritePixels(SurfaceData);
}

void UVisionComponent::TickComponent(float DeltaTime,
                                     enum ELevelTick TickType,
                                     FActorComponentTickFunction *TickFunction)
{
    Super::TickComponent(DeltaTime, TickType, TickFunction);
    // Check if paused
	if (Paused)
	{
		return;
	}

	// Check for framerate
	TimePassed += DeltaTime;
	if (!UseEngineFramerate && TimePassed < FrameTime)
	{
		return;
	}
	TimePassed -= FrameTime;
	MEASURE_TIME("Tick");

	if(!Priv || !Priv->Buffer )
	{
		return;
	}

	// ColorAllObjects();

    auto owner = GetOwner();
	owner->UpdateComponentTransforms();

	FDateTime Now = FDateTime::UtcNow();
	Priv->Buffer->HeaderWrite->TimestampCapture = Now.ToUnixTimestamp() * 1000000000 + Now.GetMillisecond() * 1000000;

	FVector Translation = GetComponentLocation();
	FQuat Rotation = GetComponentQuat();
	//Convert to meters and ROS coordinate system
	Priv->Buffer->HeaderWrite->Translation.X = Translation.X / 100.0f;
	Priv->Buffer->HeaderWrite->Translation.Y = -Translation.Y / 100.0f;
	Priv->Buffer->HeaderWrite->Translation.Z = Translation.Z / 100.0f;
	Priv->Buffer->HeaderWrite->Rotation.X = -Rotation.X;
	Priv->Buffer->HeaderWrite->Rotation.Y = Rotation.Y;
	Priv->Buffer->HeaderWrite->Rotation.Z = -Rotation.Z;
	Priv->Buffer->HeaderWrite->Rotation.W = Rotation.W;

	// Start writing to buffer
	Priv->Buffer->StartWriting(ObjectToColor, ObjectColors);

	Priv->WaitGT.lock();

	Priv->WaitGT.unlock();
	Priv->DoGT = true;
	Priv->CVGT.notify_one();

	// Read color image and notify processing thread
	Priv->WaitColor.lock();

	if(CustomColorRenderTarget)
		ReadImage(CustomColorRenderTarget, ImageColor);
	else
		ReadImage(Color->TextureTarget, ImageColor);

	FString GroundTruthData;

	Object->CaptureScene();

	FTextureRenderTargetResource *OriginalRenderTargetResource = Object->TextureTarget->GameThread_GetRenderTargetResource();
	FReadSurfaceDataFlags ReadSurfaceDataFlags;
	ReadSurfaceDataFlags.SetLinearToGamma(false);
	ReadSurfaceDataFlags.SetOutputStencil(false);
	ReadSurfaceDataFlags.SetMip(0);

	TArray<FColor> OriginalSurfaceData;
	OriginalRenderTargetResource->ReadPixels(OriginalSurfaceData, ReadSurfaceDataFlags);


	for(int32 tagIdx = 0; tagIdx < TagsToTrack.Num(); tagIdx++)
	{
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsWithTag(GetWorld(), TagsToTrack[tagIdx], FoundActors);

		if (FoundActors.Num() == 0)
		{
			FString FrameName = FString::Printf(TEXT("f%d"), FrameNumber);
			FString FrameAnnotation = FString::Printf(TEXT("{\"annotations\": []}"));

			if (GroundTruthData.IsEmpty())
			{
				GroundTruthData = FString::Printf(TEXT("{\"frameAnnotations\": {\"%s\": %s}}"), *FrameName, *FrameAnnotation);
			}
			else
			{
				GroundTruthData = FString::Printf(TEXT("%s, \"%s\": %s}"), *GroundTruthData, *FrameName, *FrameAnnotation);
			}
		}
		else
		{
			for(int32 ActorIdx = 0; ActorIdx < FoundActors.Num(); ActorIdx++)
			{
				TArray<AActor*> NewHidden;
				NewHidden.Add(FoundActors[ActorIdx]);
				
				Object->HiddenActors = NewHidden;

				Object->CaptureScene();

				//pretty bad and cheap hack to fix the offset by one frame in the ground truth data
				// FBox2D Bounds = GetActorBoxFromRT(OriginalSurfaceData, Object->TextureTarget);
				FBox2D Bounds = FBox2D();

				if (FrameNumber != 0)
				{
					Bounds = NextFrameGT;
				}
				
				NextFrameGT = GetActorBoxFromRT(OriginalSurfaceData, Object->TextureTarget);
			
				// Create ground truth data string
				FString FrameName = FString::Printf(TEXT("f%d"), FrameNumber);
				FString Annotation = FString::Printf(TEXT("{\"class\": \"%s\", \"shape\": {\"data\": [%f, %f, %f, %f], \"type\": \"bbox_xywh\"}}"), *TagsToTrack[tagIdx].ToString(), Bounds.Min.X, Bounds.Min.Y, Bounds.Max.X - Bounds.Min.X, Bounds.Max.Y - Bounds.Min.Y);
				FString FrameAnnotation = FString::Printf(TEXT("{\"annotations\": [%s]}"), *Annotation);

				if (GroundTruthData.IsEmpty())
				{
					GroundTruthData = FString::Printf(TEXT("{\"frameAnnotations\": {\"%s\": %s}}"), *FrameName, *FrameAnnotation);
				}
				else
				{
					GroundTruthData = FString::Printf(TEXT("%s, \"%s\": %s}"), *GroundTruthData, *FrameName, *FrameAnnotation);
				}

				//Clear hidden actors list
				Object->HiddenActors.Empty();
			}
		}
	}

	FrameNumber++;

	// Object->SetWorldLocationAndRotation(Color->GetComponentLocation(), Color->GetComponentRotation());
	
	Priv->WaitColor.unlock();
	Priv->DoColor = true;
	Priv->CVColor.notify_one();

	// Read object image and notify processing thread
	Priv->WaitObject.lock();

	ReadImage(Object->TextureTarget, ImageObject);

	Priv->WaitObject.unlock();
	Priv->DoObject = true;
	Priv->CVObject.notify_one();

	/* Read depth image and notify processing thread. Depth processing is called last,
	 * because the color image processing thread take more time so they can already begin.
	 * The depth processing thread will wait for the others to be finished and then releases
	 * the buffer.
	 */
	Priv->WaitDepth.lock();
	ReadImage(Depth->TextureTarget, ImageDepth);
	Priv->WaitDepth.unlock();
	Priv->DoDepth = true;
	Priv->CVDepth.notify_one();

	Priv->Buffer->StartReading();
	uint32_t xSize = Priv->Buffer->HeaderRead->Size;
	uint32_t xSizeHeader = Priv->Buffer->HeaderRead->SizeHeader; // Size of the header
	uint32_t xMapEntries = Priv->Buffer->HeaderRead->MapEntries; // Number of map entries at the end of the packet
	uint32_t xWidth = Priv->Buffer->HeaderRead->Width; // Width of the images
	uint32_t xHeight = Priv->Buffer->HeaderRead->Height; // Height of the images

	// Get the data offsets for the different types of images that are in the buffer
	const uint32_t& OffsetColor = Priv->Buffer->OffsetColor;
	const uint32_t& OffsetDepth = Priv->Buffer->OffsetDepth;
	const uint32_t& OffsetObject = Priv->Buffer->OffsetObject;
	// * - Depth image data (width * height * 2 Bytes (Float16))
	uint8_t* DepthPtr = &Priv->Buffer->Read[OffsetDepth];
	uint32_t TargetDepthBufSize = Width*Height * 4;
	uint8_t* TargetDepthBuf = new uint8_t[TargetDepthBufSize]; // Allocate a byte for every pixel * 4 Bytes for a single 32Bit Float

	const uint32_t ColorImageSize = Width * Height * 3;
	convertDepth((uint16_t *)DepthPtr, (__m128*)TargetDepthBuf);
	// convertDepth((uint16_t *)packet.pDepth, (__m128*)&msgDepth->data[0]);

	UE_LOG(LogTemp, Verbose, TEXT("Buffer Offsets: %d %d %d"), OffsetColor, OffsetDepth, OffsetObject);

	FROSTime time = FROSTime::Now();

	TSharedPtr<ROSMessages::sensor_msgs::Image> ObjectsMessage(new ROSMessages::sensor_msgs::Image());

	TSharedPtr<ROSMessages::sensor_msgs::Image> DepthMessage(new ROSMessages::sensor_msgs::Image());

	TSharedPtr<ROSMessages::sensor_msgs::Image> ImageMessage(new ROSMessages::sensor_msgs::Image());


	TSharedPtr<ROSMessages::std_msgs::String> GroundTruthMessage(new ROSMessages::std_msgs::String(GroundTruthData));
	GroundTruthPublisher->Publish(GroundTruthMessage);

	TSharedPtr<ROSMessages::std_msgs::String> MsgCountMessage(new ROSMessages::std_msgs::String(FString::FromInt(FrameNumber - 1)));
	MsgCountPublisher->Publish(MsgCountMessage);

	ObjectsMessage->header.seq = 0;
	ObjectsMessage->header.time = time;
	ObjectsMessage->header.frame_id = ImageOpticalFrame;
	ObjectsMessage->height = Height;
	ObjectsMessage->width = Width;
	ObjectsMessage->encoding = TEXT("bgr8");
	ObjectsMessage->step = Width * 3;
	ObjectsMessage->data = &Priv->Buffer->Read[OffsetObject];
	ObjectsPublisher->Publish(ObjectsMessage);

	ImageMessage->header.seq = 0;
	ImageMessage->header.time = time;
	ImageMessage->header.frame_id = ImageOpticalFrame;
	ImageMessage->height = Height;
	ImageMessage->width = Width;
	ImageMessage->encoding = TEXT("bgr8");
	ImageMessage->step = Width * 3;
	ImageMessage->data = &Priv->Buffer->Read[OffsetColor];
	ImagePublisher->Publish(ImageMessage);

	DepthMessage->header.seq = 0;
	DepthMessage->header.time = time;
	DepthMessage->header.frame_id = ImageOpticalFrame;
	DepthMessage->height = Height;
	DepthMessage->width = Width;
	DepthMessage->encoding = TEXT("32FC1");
	DepthMessage->step = Width * 4;
	DepthMessage->data = TargetDepthBuf;
	DepthPublisher->Publish(DepthMessage);

	Priv->Buffer->DoneReading();
}

void UVisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
    Running = false;

    // Stopping processing threads
    Priv->DoColor = true;
    Priv->DoDepth = true;
    Priv->DoObject = true;
	Priv->DoGT = true;
    Priv->CVColor.notify_one();
    Priv->CVDepth.notify_one();
    Priv->CVObject.notify_one();
	Priv->CVGT.notify_one();

    Priv->ThreadColor.join();
    Priv->ThreadDepth.join();
    Priv->ThreadObject.join();
	Priv->ThreadGT.join();
}

void UVisionComponent::ShowFlagsBasicSetting(FEngineShowFlags &ShowFlags) const
{
	ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_All0);
	ShowFlags.SetRendering(true);
	ShowFlags.SetStaticMeshes(true);
	ShowFlags.SetLandscape(true);
	ShowFlags.SetInstancedFoliage(true);
	ShowFlags.SetInstancedGrass(true);
	ShowFlags.SetInstancedStaticMeshes(true);
}

void UVisionComponent::ShowFlagsLit(FEngineShowFlags &ShowFlags) const
{
	ShowFlagsBasicSetting(ShowFlags);
	ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_Game);
	ApplyViewMode(VMI_Lit, true, ShowFlags);
	ShowFlags.SetMaterials(true);
	ShowFlags.SetLighting(true);
	ShowFlags.SetPostProcessing(true);
	// ToneMapper needs to be enabled, otherwise the screen will be very dark
	ShowFlags.SetTonemapper(true);
	// TemporalAA needs to be disabled, otherwise the previous frame might contaminate current frame.
	// Check: https://answers.unrealengine.com/questions/436060/low-quality-screenshot-after-setting-the-actor-pos.html for detail
	ShowFlags.SetTemporalAA(false);
	ShowFlags.SetAntiAliasing(true);
	ShowFlags.SetEyeAdaptation(false); // Eye adaption is a slow temporal procedure, not useful for image capture
}

void UVisionComponent::ShowFlagsVertexColor(FEngineShowFlags &ShowFlags) const
{
	ShowFlagsLit(ShowFlags);
	ApplyViewMode(VMI_Lit, true, ShowFlags);

	// From MeshPaintEdMode.cpp:2942
	ShowFlags.SetMaterials(false);
	ShowFlags.SetLighting(false);
	ShowFlags.SetBSPTriangles(true);
	ShowFlags.SetVertexColors(true);
	ShowFlags.SetPostProcessing(false);
	ShowFlags.SetHMDDistortion(false);
	ShowFlags.SetTonemapper(false); // This won't take effect here

	GVertexColorViewMode = EVertexColorViewMode::Color;
}

void UVisionComponent::ReadImage(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const
{
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	RenderTargetResource->ReadFloat16Pixels(ImageData);
}

void UVisionComponent::ReadImageCompressed(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const
{
	TArray<FFloat16Color> RawImageData;
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	RenderTargetResource->ReadFloat16Pixels(RawImageData);

	static IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	static TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
	ImageWrapper->SetRaw(RawImageData.GetData(), RawImageData.GetAllocatedSize(), Width, Height, ERGBFormat::BGRA, 8);
}

void UVisionComponent::ToColorImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const
{
	const FFloat16Color *itI = ImageData.GetData();
	uint8_t *itO = Bytes;

	// Converts Float colors to bytes
	for (size_t i = 0; i < ImageData.Num(); ++i, ++itI, ++itO)
	{
		*itO = (uint8_t)std::round(FMath::Clamp((float)itI->B * 255.f, 0.f, 255.f));
		*++itO = (uint8_t)std::round(FMath::Clamp((float)itI->G * 255.f, 0.f, 255.f));
		*++itO = (uint8_t)std::round(FMath::Clamp((float)itI->R * 255.f, 0.f, 255.f));
	}
	return;
}

void UVisionComponent::ToDepthImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const
{
	const FFloat16Color *itI = ImageData.GetData();
	uint16_t *itO = reinterpret_cast<uint16_t *>(Bytes);

	// Just copies the encoded Float16 values
	for (size_t i = 0; i < ImageData.Num(); ++i, ++itI, ++itO)
	{
		*itO = itI->R.Encoded;
	}
	return;
}

void UVisionComponent::StoreImage(const uint8 *ImageData, const uint32 Size, const char *Name) const
{
	std::ofstream File(Name, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
	File.write(reinterpret_cast<const char *>(ImageData), Size);
	File.close();
	return;
}

/* Generates at least NumberOfColors different colors.
 * It takes MaxHue different Hue values and additional steps ind Value and Saturation to get
 * the number of needed colors.
 */
void UVisionComponent::GenerateColors(const uint32_t NumberOfColors)
{
	const int32_t MaxHue = 50;
	// It shifts the next Hue value used, so that colors next to each other are not very similar. This is just important for humans
	const int32_t ShiftHue = 21;
	const float MinSat = 0.65;
	const float MinVal = 0.65;

	uint32_t HueCount = MaxHue;
	uint32_t SatCount = 1;
	uint32_t ValCount = 1;

	// Compute how many different Saturations and Values are needed
	int32_t left = std::max<int32_t>(0, NumberOfColors - HueCount);
	while (left > 0)
	{
		if (left > 0)
		{
			++ValCount;
			left = NumberOfColors - SatCount * ValCount * HueCount;
		}
		if (left > 0)
		{
			++SatCount;
			left = NumberOfColors - SatCount * ValCount * HueCount;
		}
	}

	const float StepHue = 360.0f / HueCount;
	const float StepSat = (1.0f - MinSat) / std::max(1.0f, SatCount - 1.0f);
	const float StepVal = (1.0f - MinVal) / std::max(1.0f, ValCount - 1.0f);

	ObjectColors.Reserve(SatCount * ValCount * HueCount);
	UE_LOG(LogTemp, Display, TEXT("Generating %d colors."), SatCount * ValCount * HueCount);

	FLinearColor HSVColor;
	for (uint32_t s = 0; s < SatCount; ++s)
	{
		HSVColor.G = 1.0f - s * StepSat;
		for (uint32_t v = 0; v < ValCount; ++v)
		{
			HSVColor.B = 1.0f - v * StepVal;
			for (uint32_t h = 0; h < HueCount; ++h)
			{
				HSVColor.R = ((h * ShiftHue) % MaxHue) * StepHue;
				ObjectColors.Add(HSVColor.HSVToLinearRGB().ToFColor(false));
				UE_LOG(LogTemp, Display, TEXT("Added color %d: %d %d %d"), ObjectColors.Num(), ObjectColors.Last().R, ObjectColors.Last().G, ObjectColors.Last().B);
			}
		}
	}
}

bool UVisionComponent::ColorObject(AActor *Actor, const FString &name)
{
	const FColor &ObjectColor = ObjectColors[ObjectToColor[name]];
	TArray<UMeshComponent *> PaintableComponents;
	Actor->GetComponents<UMeshComponent>(PaintableComponents);

	for (auto MeshComponent : PaintableComponents)
	{
		if (MeshComponent == nullptr)
			continue;

		if (UStaticMeshComponent *StaticMeshComponent = Cast<UStaticMeshComponent>(MeshComponent))
		{
			if (UStaticMesh *StaticMesh = StaticMeshComponent->GetStaticMesh())
			{
				uint32 PaintingMeshLODIndex = 0;
				uint32 NumLODLevel = StaticMesh->GetRenderData()->LODResources.Num();
				//check(NumLODLevel == 1);
				FStaticMeshLODResources &LODModel = StaticMesh->GetRenderData()->LODResources[PaintingMeshLODIndex];
				FStaticMeshComponentLODInfo *InstanceMeshLODInfo = NULL;

				// PaintingMeshLODIndex + 1 is the minimum requirement, enlarge if not satisfied
				StaticMeshComponent->SetLODDataCount(PaintingMeshLODIndex + 1, StaticMeshComponent->LODData.Num());
				InstanceMeshLODInfo = &StaticMeshComponent->LODData[PaintingMeshLODIndex];

				{
					InstanceMeshLODInfo->OverrideVertexColors = new FColorVertexBuffer;

					FColor FillColor = FColor(255, 255, 255, 255);
					InstanceMeshLODInfo->OverrideVertexColors->InitFromSingleColor(FColor::White, LODModel.GetNumVertices());
				}

				uint32 NumVertices = LODModel.GetNumVertices();

				for (uint32 ColorIndex = 0; ColorIndex < NumVertices; ++ColorIndex)
				{
					uint32 NumOverrideVertexColors = InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices();
					uint32 NumPaintedVertices = InstanceMeshLODInfo->PaintedVertices.Num();
					InstanceMeshLODInfo->OverrideVertexColors->VertexColor(ColorIndex) = ObjectColor;
				}
				BeginInitResource(InstanceMeshLODInfo->OverrideVertexColors);
				StaticMeshComponent->MarkRenderStateDirty();
			}
		}
	}
	return true;
}

bool UVisionComponent::ColorAllObjects()
{
	uint32_t NumberOfActors = 0;

	for (TActorIterator<AActor> ActItr(GetWorld()); ActItr; ++ActItr)
	{
		++NumberOfActors;
		FString ActorName = ActItr->GetHumanReadableName();
		UE_LOG(LogTemp, Display, TEXT("Actor with name: %s."), *ActorName);
	}

	UE_LOG(LogTemp, Display, TEXT("Found %d Actors."), NumberOfActors);
	GenerateColors(NumberOfActors * 2);

	for (TActorIterator<AActor> ActItr(GetWorld()); ActItr; ++ActItr)
	{
		FString ActorName = ActItr->GetHumanReadableName();
		if (!ObjectToColor.Contains(ActorName))
		{
			check(ColorsUsed < (uint32)ObjectColors.Num());
			ObjectToColor.Add(ActorName, ColorsUsed);
			UE_LOG(LogTemp, Display, TEXT("Adding color %d for object %s."), ColorsUsed, *ActorName);

			++ColorsUsed;
		}

		UE_LOG(LogTemp, Display, TEXT("Coloring object %s."), *ActorName);
		ColorObject(*ActItr, ActorName);
	}

	return true;
}

void UVisionComponent::ProcessColor()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitColor);
		Priv->CVColor.wait(WaitLock, [this] {return Priv->DoColor; });
		Priv->DoColor = false;
		if (!this->Running) break;
		ToColorImage(ImageColor, Priv->Buffer->Color);

		Priv->DoneColor = true;
		Priv->CVDone.notify_one();
	}
}

void UVisionComponent::ProcessGT()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitGT);
		Priv->CVGT.wait(WaitLock, [this] {return Priv->DoGT; });
		Priv->DoGT = false;
		if (!this->Running) break;
		Priv->DoneGT = true;
		Priv->CVDone.notify_one();
	}
}

void UVisionComponent::ProcessDepth()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitDepth);
		Priv->CVDepth.wait(WaitLock, [this] {return Priv->DoDepth; });
		Priv->DoDepth = false;
		if (!this->Running) break;
		ToDepthImage(ImageDepth, Priv->Buffer->Depth);

		// Wait for both other processing threads to be done.
		std::unique_lock<std::mutex> WaitDoneLock(Priv->WaitDone);
		Priv->CVDone.wait(WaitDoneLock, [this] {return Priv->DoneColor && Priv->DoneObject; });

		Priv->DoneColor = false;
		Priv->DoneObject = false;

		// Complete Buffer
		Priv->Buffer->DoneWriting();
	}
}

void UVisionComponent::ProcessObject()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitObject);
		Priv->CVObject.wait(WaitLock, [this] {return Priv->DoObject; });
		Priv->DoObject = false;
		if (!this->Running) break;
		ToColorImage(ImageObject, Priv->Buffer->Object);

		Priv->DoneObject = true;
		Priv->CVDone.notify_one();
	}
}

void UVisionComponent::convertDepth(const uint16_t *in, __m128 *out) const
{
	const size_t size = (Width * Height) / 4;
	for (size_t i = 0; i < size; ++i, in += 4, ++out)
	{
		// Divide by 100 here in order to convert UU (cm) into ROS units (m)
		float f0 = static_cast<float>(*(in + 0)) / 100.0f;
		float f1 = static_cast<float>(*(in + 1)) / 100.0f;
		float f2 = static_cast<float>(*(in + 2)) / 100.0f;
		float f3 = static_cast<float>(*(in + 3)) / 100.0f;
		*out = _mm_set_ps(f3, f2, f1, f0);
	}
}
