// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/DepthCameraSensor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHICommandList.h"
#include "Kismet/GameplayStatics.h"
#include "auto_scene_gen_logging.h"

UDepthCameraSensor::UDepthCameraSensor() 
{

}

void UDepthCameraSensor::InitTextureTarget(int32 Width, int32 Height) 
{
    if (ImageWidth <= 0 || ImageHeight <= 0)
	{
		UE_LOG(LogASG, Warning, TEXT("Both frame width and height must be positive integers."));
		return;
	}
    
    ImageWidth = Width;
    ImageHeight = Height;
    FOVAngle = FieldOfView;
    
    // Create TextureRenderTarget
    TextureTarget = NewObject<UTextureRenderTarget2D>();
    
    TextureTarget->InitCustomFormat(ImageWidth, ImageHeight, PF_FloatRGBA, false); // Is PF_FloatRGBA correct?
    // TextureTarget->InitAutoFormat(ImageWidth, ImageHeight);
    TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
    TextureTarget->bGPUSharedFlag = true; // Demand buffer on GPU?

	// Set Camera Properties
    CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
    bAlwaysPersistRenderingState = true;

    bInitTextureTarget = true;
}

// void UDepthCameraSensor::CaptureFloat16Color(TArray<FFloat16Color> &ImageData) 
// {
//     if (!bInitTextureTarget)
//     {
//         UE_LOG(LogASG, Error, TEXT("TextureTarget is not initialized."));
//         return;
//     }

//     CaptureScene();
//     FTextureRenderTargetResource* RenderTargetResource = TextureTarget->GameThread_GetRenderTargetResource();
//     if (!RenderTargetResource)
//     {
//         UE_LOG(LogASG, Error, TEXT("RenderTargetResource returned nullptr."));
//         return;
//     }
//     RenderTargetResource->ReadFloat16Pixels(ImageData);
// }
