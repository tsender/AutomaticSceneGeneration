// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/BaseCameraSensor.h"
#include "DepthCameraSensor.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UDepthCameraSensor : public UBaseCameraSensor
{
	GENERATED_BODY()

public:
	UDepthCameraSensor();

public: /****************************** UBaseCameraSensor Overrides ******************************/

	virtual void InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV) override;
};
