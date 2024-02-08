// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "BaseSensor.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class AUTOMATICSCENEGENERATION_API UBaseSensor : public USceneComponent
{
	GENERATED_BODY()

public:	
	UBaseSensor() {}

protected: /****************************** USceneComponent Overrides ******************************/
	virtual void BeginPlay() override
	{
		Super::BeginPlay();
		bEnabled = true;
	}

public:	/****************************** UBaseSensor ******************************/
	void Enable(bool bEnable)
	{
		bEnabled = bEnable;
	}

	bool IsEnabled() const
	{
		return bEnabled;
	}

public:	/****************************** UBaseSensor ******************************/
	UPROPERTY(EditAnywhere)
	// rosbridge server ID to connect to
	uint8 ROSBridgeServerID = 0;

protected: /****************************** UBaseSensor ******************************/
	bool bEnabled = true;
};
