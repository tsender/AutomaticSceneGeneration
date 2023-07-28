// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/BaseSensor.h"
// #include "Camera/CameraComponent.h"
#include <vector>
#include "CompleteCameraSensor.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class AUTOMATICSCENEGENERATION_API UCompleteCameraSensor : public UBaseSensor
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UCompleteCameraSensor();

protected: /****************************** UBaseSensor Overrides ******************************/
	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	/****************************** UBaseSensor Overrides ******************************/
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private: /****************************** UCompleteCameraSensor ******************************/
	UPROPERTY()
	class UColorCameraSensor* ColorCamera;

	UPROPERTY()
	class UDepthCameraSensor* DepthCamera;

	UPROPERTY()
	class USegmentationCameraSensor* TravCamera;

	UPROPERTY()
	class USegmentationCameraSensor* SegCamera;

	UPROPERTY(EditAnywhere)
	struct FPostProcessSettings ColorCameraPostProcessSettings;

	UPROPERTY(EditAnywhere)
	int32 ImageWidth = 640;

	UPROPERTY(EditAnywhere)
	int32 ImageHeight = 480;

	UPROPERTY(EditAnywhere)
	float FieldOfView = 90.f;

	UPROPERTY(EditAnywhere)
	FString SensorName = TEXT("camera");

	UPROPERTY(EditAnywhere)
	bool bSaveImagesToDisk = false;
	
	UPROPERTY(EditAnywhere)
	FString SaveFolder = TEXT("TrainingData");

	UPROPERTY(EditAnywhere)
	float FrameRate = 15.f;

	UPROPERTY(EditAnywhere)
	bool bEnableDepthCam = false;

	UPROPERTY(EditAnywhere)
	bool bEnableTravCam = false;

	UPROPERTY(EditAnywhere)
	bool bEnableSegCam = false;

	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	UPROPERTY()
	class UTopic* ColorCamPub;

	UPROPERTY()
	class UTopic* DepthCamPub;

	UPROPERTY()
	class UTopic* TravCamPub;
	
	UPROPERTY()
	class UTopic* SegCamPub;

	FString HeaderFrameID;
	int HeaderSequence = 1;

	FString CameraFolder;
	FString FrameNumberPath;

	// Vectors to store raw/encoded image data
	std::vector<uint8> EncodedColorData;
	std::vector<uint8> EncodedDepthData;
	std::vector<uint8> EncodedTravData;
	std::vector<uint8> EncodedSegData;
	
	void TickSensor();
};
