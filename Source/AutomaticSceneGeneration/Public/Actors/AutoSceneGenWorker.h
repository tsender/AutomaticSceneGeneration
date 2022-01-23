// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"
#include <chrono>
#include "AutoSceneGenWorker.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogASG, Log, All);

UCLASS()
class AUTOMATICSCENEGENERATION_API AAutoSceneGenWorker : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AAutoSceneGenWorker();

protected: /****************************** AActor Overrides ******************************/
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	/****************************** AActor Overrides ******************************/
	virtual void Tick(float DeltaTime) override;

public: /****************************** AAutoSceneGenWorker ******************************/
	uint8 GetWorkerID() const;
	
	TArray<float> GetSSAAttributes(uint16 Subclass, uint16 index) const;

private: /****************************** AAutoSceneGenWorker ******************************/
	// ODD variables/functions //////////////////////////////////////////////////////////////////////////////
	// Structural scene actor data array
	UPROPERTY()
	TArray<float> SSADataArray;
	
	// Array of all structural scene actors in the scene
	UPROPERTY()
	TArray<class AStructuralSceneActor*> StructuralSceneActorArray;

	// Structural scene actor subclasses that will be placed in the scene
	UPROPERTY(EditAnywhere)
	TArray<TSubclassOf<class AStructuralSceneActor>> StructuralSceneActorSubclasses;

	UPROPERTY()
	class AStaticMeshActor* GroundPlaneActor;

	// Number of structural scene actor instances allowed per type
	UPROPERTY(EditAnywhere)
	uint16 NumSSAInstances = 1000;

	// ASG Worker ID number
	UPROPERTY(EditAnywhere)
	uint8 WorkerID = 0;

	// UPROPERTY(EditAnywhere)
	// float GlobalTimeDilation = 1.f;

	uint16 NumSSASubclasses;

	uint16 SSADataArraySize;

	bool bSSAInit = false;

	float GroundPlaneZHeight = 0.f;

	// Radius [cm] around the start/goal points from which no structural scene actors can be placed
	UPROPERTY(EditAnywhere)
	float SafetyRadius = 500.f;
	
	UPROPERTY(EditAnywhere)
	FVector LandscapeSize = FVector(50000., 50000., 0.);

	// Evaluation vehicle
	UPROPERTY()
	class AAutoSceneGenVehicle* ASGVehicle;

	// The trigger volume is used to determine the end of an experiment (i.e. when the vehicle overlaps it)
	UPROPERTY(EditAnywhere)
	class ATriggerVolume* TriggerVolume;

	UPROPERTY()
	class APlayerStart* PlayerStart;

	// ROSIntegration game instance
	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;
	
	// ROS subscriber: Subscribes to the ASG's status
	UPROPERTY()
	class UTopic* ASGStatusSub;

	// ROS publisher: Publishes the status of the ASG worker
	UPROPERTY()
	class UTopic* WorkerStatusPub;

	// ROS publisher: Publishes the destination for the vehicle
	UPROPERTY()
	class UTopic* VehicleDestinationPub;

	// ROS service: Processed the RunScenario request from the ASG
	UPROPERTY()
	class UService* RunScenarioService;

	// ROS client: Sends the scenario data to the ASG for its analysis
	UPROPERTY()
	class UService* AnalyzeScenarioClient;

	/**
	 *  If the duration between consecutive ASG status messages reaches this threshold,
	 * 	then reset the simulation and wait for the ASG to come back online before restarting the run.
	 */
	UPROPERTY(EditAnywhere)
	float ASGStatusMessagePeriodThreshold = 5.f;

	bool bASGOnline = false;

	bool bASGStatusClockInit = false;

	std::chrono::steady_clock::time_point LastASGStatusClockTime;

	uint8 WorkerStatus = 0;

	int32 ScenarioNumber = 0;

	bool bForceVehicleReset = false;

	bool bWaitingForScenarioRequest = false;

	bool bProcessedScenarioRequest = true;

	bool bReadyToTick = false;

	bool bDoneTesting = false;

	void InitStructuralSceneActorArray();

	void RandomizeStructuralSceneActors();

	bool CheckIfVehicleCrashed();

	bool CheckIfVehicleTurnedOver();

	bool CheckTriggerVolume();

	void ProcessScenarioRequest();

	void ASGStatusCB(TSharedPtr<FROSBaseMsg> Msg);

	/**
	 * ROS callback for receiving the ASG client's response for the most recent scenario analysis
	 * @param Response The response message from the ASG client
	 */
	void AnalyzeScenarioResponseCB(TSharedPtr<FROSBaseServiceResponse> Response);

	/**
	 * ROS callback for receiving the new scenario description from the ASG client
	 * @param Request The request message from the ASG client containing the scenario description
	 * @param Response The response message to be sent to the ASG client (indicates receipt of request)
	 */
	void RunScenarioServiceCB(TSharedPtr<FROSBaseServiceRequest> Request, TSharedPtr<FROSBaseServiceResponse> Response);
};
