// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/DepthCameraSensor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHICommandList.h"
#include "Kismet/GameplayStatics.h"
#include "AutoSceneGenLogging.h"

UDepthCameraSensor::UDepthCameraSensor() 
{

}

void UDepthCameraSensor::InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV) 
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
    
    TextureTarget->InitCustomFormat(ImageWidth, ImageHeight, PF_FloatRGBA, false); // Is PF_FloatRGBA correct?
    TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
    TextureTarget->bGPUSharedFlag = true; // Demand buffer on GPU?

	// Set Camera Properties
    CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
    bAlwaysPersistRenderingState = true;

    bInitTextureTarget = true;
}