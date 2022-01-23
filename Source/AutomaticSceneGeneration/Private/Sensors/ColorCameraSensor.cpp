// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/ColorCameraSensor.h"
#include "Engine/TextureRenderTarget2D.h"

UColorCameraSensor::UColorCameraSensor() 
{

}

void UColorCameraSensor::InitTextureTarget(int32 Width, int32 Height) 
{
    Super::InitTextureTarget(Width, Height);
    TextureTarget->TargetGamma = 1.2f;
}
