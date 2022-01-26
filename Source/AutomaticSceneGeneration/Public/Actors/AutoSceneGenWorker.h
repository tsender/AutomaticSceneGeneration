// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"
#include <chrono>
#include "AutoSceneGenWorker.generated.h"

USTRUCT()
struct FStructuralSceneActorPtrs
{
	GENERATED_BODY()
	
	UPROPERTY()
	TArray<class AStructuralSceneActor*> Ptrs;

	FString SSASubclass;

	FStructuralSceneActorPtrs() {}

	FStructuralSceneActorPtrs(FString InSSASubclass)
	{
		SSASubclass = InSSASubclass;
	}
	
	~FStructuralSceneActorPtrs() 
	{
		Ptrs.Empty();
	}
};

struct FStructuralSceneActorAttrs
{
	TArray<float> Attrs;
	int32 NumSSAs;

	FStructuralSceneActorAttrs(TArray<float> InAttrs, int32 InNumSSAs)
	{
		Attrs = InAttrs;
		NumSSAs = InNumSSAs;
	}

	~FStructuralSceneActorAttrs() 
	{
		Attrs.Empty();
	}
};

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
	
	// Array of pointers to all structural scene actors in the scene
	UPROPERTY()
	TArray<class AStructuralSceneActor*> StructuralSceneActorArray;

	// Structural scene actor subclasses that will be placed in the scene
	UPROPERTY(EditAnywhere)
	TArray<TSubclassOf<class AStructuralSceneActor>> StructuralSceneActorSubclasses;

	UPROPERTY()
	TMap<FString, FStructuralSceneActorPtrs> SSAPtrMap;

	TMap<FString, FStructuralSceneActorAttrs> SSAAttrrMap;

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

	// For now, we assume the ground plane is flat
	float GroundPlaneZHeight = 0.f;

	// The dimensions of the landscape in [cm]
	UPROPERTY(EditAnywhere)
	FVector LandscapeSize = FVector(50000., 50000., 0.);

	// Radius [cm] around the start/goal points from which no structural scene actors can be placed. This is only used when creating scenes randomly.
	UPROPERTY(EditAnywhere)
	float SafetyRadius = 500.f; // [cm]

	/**
	 * If vehicle is within this distance in [cm] from the goal point, then the vehicle has reached its destination. 
	 * This distance accounts for the turning radius of the vehicle. Without this, then the vehicle may end up circling the goal point for a really long time, which we don't want.
	 */
	UPROPERTY(EditAnywhere)
	float GoalRadius = 5000.f; // [cm]
	
	// The starting point for the vehicle in [cm]
	UPROPERTY(EditAnywhere)
	FVector VehicleStartLocation = FVector(10000., -10000., 200.);

	// The starting yaw angle for the vehicle in [deg]
	UPROPERTY(EditAnywhere)
	float VehicleStartYaw = -45.f;

	FRotator VehicleStartRotation = FRotator(0.f, 0.f, 0.f);

	// The goal point for the vehicle in [cm]
	UPROPERTY(EditAnywhere)
	FVector VehicleGoalLocation = FVector(30000., -30000., 0.);

	// Maximum allowed vehicle roll angle [deg]
	UPROPERTY(EditAnywhere)
	float MaxVehicleRoll = 70.f;
	
	// Maximum allowed vehicle pitch angle [deg]
	UPROPERTY(EditAnywhere)
	float MaxVehiclePitch = 70.f;

	// Evaluation vehicle
	UPROPERTY()
	class AAutoSceneGenVehicle* ASGVehicle;

	// ROSIntegration game instance
	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;
	
	// ROS topic name for ASG client's status
	UPROPERTY(Editanywhere)
	FString ASGClientStatusTopic = FString("/asg_client/status");

	// ROS subscriber: Subscribes to the ASG client's status
	UPROPERTY()
	class UTopic* ASGClientStatusSub;

	// ROS publisher: Publishes the status of the ASG worker
	UPROPERTY()
	class UTopic* WorkerStatusPub;

	// ROS publisher: Publishes the destination for the vehicle
	UPROPERTY()
	class UTopic* VehicleDestinationPub;

	// ROS service: Processes the RunScenario request from the ASG client
	UPROPERTY()
	class UService* RunScenarioService;

	// ROS client: Sends the scenario data to the ASG for its analysis
	UPROPERTY()
	class UService* AnalyzeScenarioClient;

	/**
	 *  If the duration between consecutive ASG client status messages reaches this threshold,
	 * 	then reset the simulation and wait for the ASG client to come back online before restarting the run.
	 */
	// UPROPERTY(EditAnywhere)
	// float ASGStatusMessagePeriodThreshold = 5.f;

	bool bASGClientOnline = false;

	// bool bASGStatusClockInit = false;

	// std::chrono::steady_clock::time_point LastASGStatusClockTime;

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

	bool CheckIfVehicleFlipped();

	bool CheckGoalLocation();

	void ProcessScenarioRequest();

	void ASGClientStatusCB(TSharedPtr<FROSBaseMsg> Msg);

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
