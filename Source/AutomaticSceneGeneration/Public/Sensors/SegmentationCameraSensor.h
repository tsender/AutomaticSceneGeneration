// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/BaseCameraSensor.h"
#include "SegmentationCameraSensor.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API USegmentationCameraSensor : public UBaseCameraSensor
{
	GENERATED_BODY()

public:
	USegmentationCameraSensor();

public: /****************************** UBaseCameraSensor Overrides ******************************/
	virtual void InitTextureTarget(int32 Width, int32 Height, float FOV) override;
};
