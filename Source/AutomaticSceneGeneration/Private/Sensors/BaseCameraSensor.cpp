// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/BaseCameraSensor.h"
#include "Components/AnnotationComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHICommandList.h"
#include "ImageWrapper/Public/IImageWrapperModule.h"
#include "ImageWrapper/Public/IImageWrapper.h"
#include "Modules/ModuleManager.h"
#include "Kismet/GameplayStatics.h"
#include "AutoSceneGenLogging.h"

UBaseCameraSensor::UBaseCameraSensor() 
{
    // Disable capture bools since we need to handle this manually
    bCaptureEveryFrame = false;
	bCaptureOnMovement = false;

    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
    ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG); //EImageFormat::PNG //EImageFormat::JPEG
    if (!ImageWrapper)
    {
        UE_LOG(LogASG, Warning, TEXT("ImageWrapper could not be initialized"));
        return;
    }
}

void UBaseCameraSensor::BeginPlay() 
{
    Super::BeginPlay();
    InitAnnotatableActors();
}

void UBaseCameraSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) 
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UBaseCameraSensor::InitAnnotatableActors() 
{
    TArray<AActor*> TempArray;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), TempArray);

	for (AActor* Actor : TempArray)
	{
		UAnnotationComponent* AnnotationComp = Cast<UAnnotationComponent>(Actor->GetComponentByClass(UAnnotationComponent::StaticClass()));
		if (AnnotationComp)
		{
			WorldAnnotationComponents.Emplace(AnnotationComp);
        }
    }
}

// These settings will be fine for most camera sensors
void UBaseCameraSensor::InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV) 
{
    if (NewWidth <= 0 || NewHeight <= 0)
	{
		UE_LOG(LogASG, Warning, TEXT("Both frame width and height must be positive integers."));
		return;
	}
    
    ImageWidth = NewWidth;
    ImageHeight = NewHeight;
    FOVAngle = NewFOV;
    
    // Create TextureRenderTarget
    if (TextureTarget)
    {
        UTextureRenderTarget2D* OldTarget = TextureTarget;
        TextureTarget = nullptr;
        OldTarget->ConditionalBeginDestroy();

    }
    TextureTarget = NewObject<UTextureRenderTarget2D>();
    
    TextureTarget->InitCustomFormat(ImageWidth, ImageHeight, PF_B8G8R8A8, false); // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
    TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
    TextureTarget->bGPUSharedFlag = true; // Demand buffer on GPU?

	// Set Camera Properties
    CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    bAlwaysPersistRenderingState = true;

    bInitTextureTarget = true;
}

void UBaseCameraSensor::ResizeTextureTarget(int32 NewWidth, int32 NewHeight)
{
    if (NewWidth <= 0 || NewHeight <= 0)
	{
		UE_LOG(LogASG, Warning, TEXT("Both frame width and height must be positive integers."));
		return;
	}

    if (TextureTarget)
        TextureTarget->ResizeTarget(NewWidth, NewHeight);
}

void UBaseCameraSensor::SetFOV(float NewFOV)
{
    FOVAngle = NewFOV;
}

int32 UBaseCameraSensor::GetImageWidth() 
{
    return ImageWidth;
}

int32 UBaseCameraSensor::GetImageHeight() 
{
    return ImageHeight;
}

void UBaseCameraSensor::SetSaveImages(bool bSaveImagesIn) 
{
    bSaveImages = bSaveImagesIn;
}

void UBaseCameraSensor::SetSavePrefix(FString NewSavePrefix) 
{
    SavePrefix = NewSavePrefix;
}

int32 UBaseCameraSensor::GetFrameNumber() const
{
    return FrameNumber;
}

void UBaseCameraSensor::SetFrameNumber(int32 NewFrameNumber)
{
    FrameNumber = NewFrameNumber;
}

void UBaseCameraSensor::CaptureColor(TArray<FColor> &ImageData, bool bUseAnnotationMaterial, uint8 AnnotationMaterialID) 
{
    if (!bInitTextureTarget)
    {
        UE_LOG(LogASG, Error, TEXT("TextureTarget is not initialized."));
        return;
    }

    FTextureRenderTargetResource* RenderTargetResource = TextureTarget->GameThread_GetRenderTargetResource();
    if (!RenderTargetResource)
    {
        UE_LOG(LogASG, Error, TEXT("RenderTargetResource returned nullptr."));
        return;
    }

    // UE_LOG(LogASG, Error, TEXT("Looping over annotation comp."));
    // Set annotation material if the object is within or just outside of the camera's FOV. A few outlier cases may arise, but for now this seems to be fine.
    FVector CamLocation = GetComponentLocation();
    float CamYaw = GetComponentRotation().Yaw * PI / 180.f;
    for (UAnnotationComponent* Comp : WorldAnnotationComponents)
    {
        // UE_LOG(LogASG, Error, TEXT("Found annotation comp."));
        if (Comp->GetOwner()->ActorHasTag("sky") || Comp->GetOwner()->ActorHasTag("ground_plane"))
        {
            Comp->SetActiveMaterial(bUseAnnotationMaterial, AnnotationMaterialID);
            continue;
        }

        // Get actor location relative to camera location and put into camera reference frame
        FVector RelLocation = Comp->GetOwner()->GetActorLocation() - CamLocation;
        float x1 = FMath::Cos(CamYaw) * RelLocation.X + FMath::Sin(CamYaw) * RelLocation.Y;
        float y1 = -FMath::Sin(CamYaw) * RelLocation.X + FMath::Cos(CamYaw) * RelLocation.Y;
        float m = FMath::Cos(FOVAngle*PI/360.f) / FMath::Sin(FOVAngle*PI/360.f);
        float xb = -1000.f / FMath::Sin(FOVAngle*PI/360.f); // 10m buffer from actual FOV

        // If (x1, y1) in field of view, then set material
        if ((x1 > -m*y1 + xb) && (x1 > m*y1 + xb))
        {
            Comp->SetActiveMaterial(bUseAnnotationMaterial, AnnotationMaterialID);
        }
    }

    CaptureScene(); // Renders scene to texture target
    FReadSurfaceDataFlags ReadSurfaceDataFlags;
	ReadSurfaceDataFlags.SetLinearToGamma(false);
    RenderTargetResource->ReadPixels(ImageData, ReadSurfaceDataFlags); // Slow, but does the job properly

    if (bSaveImages)
    {
        FFrameData* FrameData = new FFrameData(ImageData, SavePrefix + FString::Printf(TEXT("_%06d.png"), FrameNumber));
        PendingSaveQueue.Enqueue(FrameData);
        FrameNumber++;
        SavePendingFrames();
    }
}

void UBaseCameraSensor::CaptureFloat16Color(TArray<FFloat16Color> &ImageData)
{
    if (!bInitTextureTarget)
    {
        UE_LOG(LogASG, Error, TEXT("TextureTarget is not initialized."));
        return;
    }

    CaptureScene();
    FTextureRenderTargetResource* RenderTargetResource = TextureTarget->GameThread_GetRenderTargetResource();
    if (!RenderTargetResource)
    {
        UE_LOG(LogASG, Error, TEXT("RenderTargetResource returned nullptr."));
        return;
    }
    RenderTargetResource->ReadFloat16Pixels(ImageData);
}

void UBaseCameraSensor::SavePendingFrames() 
{
    while (!PendingSaveQueue.IsEmpty()){
        // Peek the next frame
        FFrameData* FrameData = nullptr;
        PendingSaveQueue.Peek(FrameData);

        if(FrameData && ImageWrapper){
            // Get compressed image data in format for saving
            ImageWrapper->SetRaw(FrameData->ImageData.GetData(), FrameData->ImageData.GetAllocatedSize(), ImageWidth, ImageHeight, ERGBFormat::BGRA, 8);
            const TArray64<uint8>& CompressedData = ImageWrapper->GetCompressed(5); //GetCompressed(5)?
            (new FAutoDeleteAsyncTask<AsyncSaveImageTask>(CompressedData, FrameData->Filename))->StartBackgroundTask();

            // Delete the first element from the queue and the dynamically allocated pointer
            PendingSaveQueue.Pop();
            delete FrameData;
        }
    }
}