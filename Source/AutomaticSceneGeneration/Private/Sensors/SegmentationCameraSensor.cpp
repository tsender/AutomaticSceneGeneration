// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/SegmentationCameraSensor.h"
#include "Engine/TextureRenderTarget2D.h"

USegmentationCameraSensor::USegmentationCameraSensor() 
{

}

void USegmentationCameraSensor::InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV) 
{
    Super::InitTextureTarget(NewWidth, NewHeight, NewFOV);
    ShowFlags.SetPostProcessing(false); // Allows for unlit colors in materials to be one solid color
    ShowFlags.SetFog(false); // Must remove fog effects
    ShowFlags.SetVolumetricFog(false); // Must remove fog effects
    ShowFlags.SetLighting(false);
	ShowFlags.SetColorGrading(false); // UnrealCV has this set to false
	ShowFlags.SetTonemapper(false); // UnrealCV says important to disable this
}
