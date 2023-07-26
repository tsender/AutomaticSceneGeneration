// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/ColorCameraSensor.h"
#include "Engine/TextureRenderTarget2D.h"

UColorCameraSensor::UColorCameraSensor() 
{

}

void UColorCameraSensor::InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV) 
{
    Super::InitTextureTarget(NewWidth, NewHeight, NewFOV);
    TextureTarget->TargetGamma = 1.2f;
}
