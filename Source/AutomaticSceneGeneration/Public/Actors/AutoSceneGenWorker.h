// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "auto_scene_gen_msgs/msg/SceneDescription.h"
#include "auto_scene_gen_msgs/msg/SceneCaptureSettings.h"
#include <chrono>
#include <vector>
#include "AutoSceneGenWorker.generated.h"

// NOTE: Make sure to uncheck EnableWorldBoundsCheck in world settings

namespace ROSMessages
{
	namespace auto_scene_gen_msgs
	{
		class FAnalyzeScenarioRequest;
	}
}


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

private: /****************************** AAutoSceneGenWorker ******************************/
	
	UPROPERTY(EditAnywhere)
	// ASG Worker ID number
	uint8 WorkerID = 0;

	UPROPERTY()
	// Perspective camera for capturing image views of the scene
	class UColorCameraSensor* PerspectiveCamera;

	UPROPERTY()
	// Orthographic camera for capturing image views of the scene
	class UColorCameraSensor* OrthoCamera;

    UPROPERTY(EditAnywhere)
	int32 CameraImageSize = 1024;

    // UPROPERTY(EditAnywhere)
	// Perspective camera field of view in [deg]
	float CameraFOV = 60.f;

	bool bTookSceneCaptureInternal = false;

	// Each key is the name of the scene capture, and each value is the raw uint8 array
	TMap<FString, std::vector<uint8>> RawSceneCaptures;

	// Stores the latest scene capture settings
	ROSMessages::auto_scene_gen_msgs::SceneCaptureSettings SceneCaptureSettings;

	// Keeps track of the requested scene description
	ROSMessages::auto_scene_gen_msgs::SceneDescription SceneDescription;

	bool bTakeSceneCapture = true;

	bool bSceneCaptureOnly = false;

	UPROPERTY()
	// Keeps track of the SSA maintainers
	TMap<FString, class UStructuralSceneActorMaintainer*> SSAMaintainerMap;

	UPROPERTY()
	class AAutoSceneGenLandscape* ASGLandscape;

	UPROPERTY(EditAnywhere)
	UMaterialInterface* LandscapeMaterial;

	UPROPERTY(EditAnywhere)
	int32 DebugLandscapeSubdivisions = 1;

	UPROPERTY(EditAnywhere)
	// The dimensions of the landscape in [cm]
	float LandscapeSize = 500. * 100.;

	UPROPERTY(EditAnywhere)
	// (Optional) landscape border in [cm]
	float LandscapeBorder = 0.;

	FBox LandscapeBox;

	UPROPERTY()
	class ADirectionalLight* LightSource;

	UPROPERTY(EditAnywhere)
	/**
	 * Structural scene actor subclasses that will be placed in the scene for debugging purposes. 
	 * They will get overwritten upon processing the new RunScenario request.
	 */
	TArray<TSubclassOf<class AStructuralSceneActor>> DebugSSASubclasses;

	UPROPERTY(EditAnywhere)
	// Number of structural scene actor instances allowed per type
	uint16 DebugNumSSAInstances = 100;

	UPROPERTY(EditAnywhere)
	// Indicates if the debug structural scene actors can cast a shadow
	bool bDebugSSACastShadow = true;

	// REMOVE: For now, we assume the ground plane is flat
	float GroundPlaneZHeight = 0.f;

	UPROPERTY(EditAnywhere)
	// Maximum amount of time [s] to let the simulation run before terminating. Set to -1 to disable feature.
	float SimTimeoutPeriod = -1.;
	
	UPROPERTY(EditAnywhere)
	/**
	 * Maximum amount of time [s] the vehicle can idle (once it began moving) before terminating the simulation. Set to -1 to disable feature.
	 * Idling is defined as being at/near rest while also commanding zero velocity.
	 */
	float VehicleIdlingTimeoutPeriod = -1.;
	
	UPROPERTY(EditAnywhere)
	/**
	 * Maximum amount of time [s] the vehicle can be "stuck", like on an obstacle, before terminating the simulation. Set to -t to disable feature.
	 * We define the vehicle as being stuck if it is not moving, has not flipped over, but is still being sent non-zero control commands.
	 */
	float VehicleStuckTimeoutPeriod = -1.;
	
	UPROPERTY(EditAnywhere)
	/**
	 * If true, then the simulator will not terminate the simulation if the vehicle touches a non-traversable obstacle.
	 * If false, then the simulation will terminate with reason REASON_VEHICLE_COLLISION (see auto_scene_gen_msgs/srv/AnalyzeScenarioRequest.h) if the vehicle touches a non-traversable obstacle.
	 */
	bool bAllowCollisions = true;

	UPROPERTY(EditAnywhere)
	// Radius [cm] around the start/goal points from which no structural scene actors can be placed. This is only used when creating scenes randomly.
	float DebugSafetyRadius = 500.f;

	UPROPERTY(EditAnywhere)
	/**
	 * If vehicle is within this distance in [cm] from the goal point, then the vehicle has reached its destination. 
	 * This distance accounts for the turning radius of the vehicle. Without this, then the vehicle may end up circling the goal point for a really long time, which we don't want.
	 */
	float GoalRadius = 500.f;
	
	UPROPERTY(EditAnywhere)
	// The starting point for the vehicle in [cm]
	FVector VehicleStartLocation = FVector(10000., -10000., 200.);

	UPROPERTY(EditAnywhere)
	// The starting yaw angle for the vehicle in [deg]
	float VehicleStartYaw = -45.f;

	FRotator VehicleStartRotation = FRotator(0.f, 0.f, 0.f);

	UPROPERTY(EditAnywhere)
	// The goal point for the vehicle in [cm]
	FVector VehicleGoalLocation = FVector(30000., -30000., 0.);

	UPROPERTY(EditAnywhere)
	// Maximum allowed vehicle roll angle [deg]
	float MaxVehicleRoll = 70.f;
	
	UPROPERTY(EditAnywhere)
	// Maximum allowed vehicle pitch angle [deg]
	float MaxVehiclePitch = 70.f;

	UPROPERTY()
	// Evaluation vehicle
	class AAutoSceneGenVehicle* ASGVehicle;

	UPROPERTY()
	// ROSIntegration game instance
	class UROSIntegrationGameInstance* ROSInst;

	bool bROSBridgeHealthy = false;

	bool bROSBridgeConnectionInterrupted = false;
	
	UPROPERTY(Editanywhere)
	// Name of the ROS AutoSceneGenClient
	FString AutoSceneGenClientName = FString("asg_client");

	UPROPERTY()
	// ROS subscriber: Subscribes to the ASG client's status
	class UTopic* ASGClientStatusSub;

	UPROPERTY()
	// ROS publisher: Publishes the status of the ASG worker
	class UTopic* WorkerStatusPub;

	UPROPERTY()
	// ROS publisher: Publishes the destination for the vehicle
	class UTopic* VehicleDestinationPub;

	UPROPERTY()
	// ROS service: Processes the RunScenario request from the ASG client
	class UService* RunScenarioService;

	UPROPERTY()
	// ROS client: Sends the scenario data to the ASG for its analysis
	class UService* AnalyzeScenarioClient;

	UPROPERTY()
	// ROS client: Sends the worker issue notification to the ASG
	class UService* WorkerIssueNotificationClient;

	UPROPERTY()
	// ROS client: Sends the scene captures to the ASG client
	class UService* SceneCaptureClient;

	bool bASGClientOnline = false;

	uint8 WorkerStatus = 0;

	int32 ScenarioNumber = 0;

	bool bForceVehicleReset = false;

	bool bWaitingForScenarioRequest = false;

	bool bProcessedScenarioRequest = true;

	bool bReadyToTick = false;

	void RandomizeDebugStructuralSceneActors();
	
	// Update the vehicle's starting Z location based on the newly generated landscape so that vehicle starts just above it
	void SetVehicleStartZLocation();

	void PublishStatus();

	/**
	 * Send a WorkerIssueNotification to the ASG client
	 * @param IssueID Issue ID
	 * @param ErrorMessage Error message to send to ASG client
	 * 
	*/
	void SendWorkerIssueNotification(uint8 IssueID, FString ErrorMessage);

	void ProcessRunScenarioRequest();

	/**
	 * Reset vehicle send an AnalyzeScenario request for the given termination reason
	 * @param TerminationReason Reason for terminating the simulation
	 */
	void ResetVehicleAndSendAnalyzeScenarioRequest(uint8 TerminationReason);

	bool CheckForVehicleReset();

	bool CheckGoalLocation();

	void OnROSConnectionStatus(bool bIsConnected);

	/**
	 * ROS callback for receiving the ASG client's status
	 * @param Msg The status message from the ASG client
	 */
	void ASGClientStatusCB(TSharedPtr<class FROSBaseMsg> Msg);

	/**
	 * ROS callback for receiving the ASG client's response for the worker issue notification
	 * @param Response The response message from the ASG client
	 */
	void WorkerIssueNotificationResponseCB(TSharedPtr<class FROSBaseServiceResponse> Response);

	/**
	 * ROS callback for receiving the ASG client's response for the most recent scenario analysis
	 * @param Response The response message from the ASG client
	 */
	void AnalyzeScenarioResponseCB(TSharedPtr<class FROSBaseServiceResponse> Response);

	/**
	 * ROS callback for receiving the new scenario description from the ASG client
	 * @param Request The request message from the ASG client containing the scenario description
	 * @param Response The response message to be sent to the ASG client (indicates receipt of request)
	 */
	void RunScenarioServiceCB(TSharedPtr<class FROSBaseServiceRequest> Request, TSharedPtr<class FROSBaseServiceResponse> Response);

	/**
	 * ROS callback for receiving the ASG client's response for the scene captures
	 * @param Response The response message from the ASG client
	 */
	void SceneCaptureResponseCB(TSharedPtr<class FROSBaseServiceResponse> Response);

	// Clear all stored scene capture data
	void ClearSceneCaptures();

	/**
	 * Extract the raw image data from the provided scene capture image
	 * @param ImageName The name of the provided image
	 * @param ImageData The actual color data from the image
	*/
	void StoreSceneCapture(FString ImageName, TArray<FColor> &ImageData);

	/**
	 * Captrue the scene images as specified in the SceneCaptureSettings message
	 * @param bDrawAnnotations Indicates if we should include the annotations specified in the SceneCaptureSettings message
	*/
	void CaptureSceneImages(bool bDrawAnnotations);

	/**
	 * Add the scene capture data to the AnalyzeScenario request
	 * @param Request The AnalyzeScenario request to add scene capture data to
	*/
	void AddSceneCapturesToRequest(TSharedPtr<ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest> Request);
};
