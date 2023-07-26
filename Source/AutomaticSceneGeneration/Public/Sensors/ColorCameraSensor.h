// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/BaseCameraSensor.h"
#include "ColorCameraSensor.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API UColorCameraSensor : public UBaseCameraSensor
{
	GENERATED_BODY()

public:
	UColorCameraSensor();

public: /****************************** UBaseCameraSensor Overrides ******************************/

	virtual void InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV) override;
};
