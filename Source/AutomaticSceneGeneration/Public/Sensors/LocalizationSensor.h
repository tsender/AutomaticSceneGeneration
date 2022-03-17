// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/BaseSensor.h"
#include "LocalizationSensor.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class AUTOMATICSCENEGENERATION_API ULocalizationSensor : public UBaseSensor
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	ULocalizationSensor();

protected: /****************************** USceneComponent Overrides ******************************/
	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	/****************************** USceneComponent Overrides ******************************/
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private: /****************************** ULocalizationSensor ******************************/	
	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	UPROPERTY()
	class UTopic* SensorPub;

	UPROPERTY(EditAnywhere)
	FString SensorName = TEXT("localization");
	
	UPROPERTY(EditAnywhere)
	float FrameRate = 60.f;

	FString HeaderFrameID;
	int HeaderSequence = 1;

	void TickSensor();
};
