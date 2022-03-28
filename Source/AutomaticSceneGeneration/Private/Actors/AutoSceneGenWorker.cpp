// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/AutoSceneGenWorker.h"
#include "Objects/StructuralSceneActorMaintainer.h"
#include "Actors/StructuralSceneActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "auto_scene_gen_logging.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/RI/Service.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/std_msgs/Bool.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"
#include "ROSIntegration/Public/geometry_msgs/Quaternion.h"
#include "ROSIntegration/Public/nav_msgs/Path.h"

#include "auto_scene_gen_msgs/msg/StatusCode.h"
#include "auto_scene_gen_msgs/srv/RunScenarioRequest.h"
#include "auto_scene_gen_msgs/srv/RunScenarioResponse.h"
#include "auto_scene_gen_msgs/srv/AnalyzeScenarioRequest.h"
#include "auto_scene_gen_msgs/srv/AnalyzeScenarioResponse.h"

// Sets default values
AAutoSceneGenWorker::AAutoSceneGenWorker()
{
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AAutoSceneGenWorker::BeginPlay()
{
	Super::BeginPlay();

	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
	ScenarioNumber = 0;
	bASGClientOnline = false;
	bForceVehicleReset = false;
	bWaitingForScenarioRequest = false;
	bProcessedScenarioRequest = true;
	bReadyToTick = false;

	// Find ground plane actor
	TArray<AActor*> TempArray;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), TempArray);
	if (!TempArray.Num())
	{
		UE_LOG(LogASG, Error, TEXT("The ground plane must be a StaticMeshActors with tag 'ground_plane', but could not find any StaticMeshActors in the scene."));
		return;
	}
	for (AActor* Actor: TempArray)
	{
		if (Actor->ActorHasTag("ground_plane"))
		{
			FVector Origin;
			FVector BoxExtent;
			Actor->GetActorBounds(true, Origin, BoxExtent);
			GroundPlaneZHeight = Origin.Z + BoxExtent.Z;
			// LandscapeSize = 2*BoxExtent;
			break;
		}
	}

	// Find ASG vehicle
	VehicleStartRotation = FRotator(0.f, VehicleStartYaw, 0.f);
	ASGVehicle = Cast<AAutoSceneGenVehicle>(GetWorld()->GetFirstPlayerController()->GetPawn());
	if (!ASGVehicle)
	{
		UE_LOG(LogASG, Error, TEXT("Could not get AAutoSceneGenVehicle from first player controller."));
	}
	else
	{
		ASGVehicle->SetDefaultResetInfo(VehicleStartLocation, VehicleStartRotation);
	}

	// Add debug structural scene actors to SSA maintainer map
	for (TSubclassOf<AStructuralSceneActor> Subclass : DebugSSASubclasses)
	{
		SSAMaintainerMap.Add(Subclass->GetPathName(), NewObject<UStructuralSceneActorMaintainer>(UStructuralSceneActorMaintainer::StaticClass()));
		SSAMaintainerMap[Subclass->GetPathName()]->Init(GetWorld(), Subclass);
	}

	ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	if (ROSInst)
	{
		// ASG client status sub
		ASGClientStatusSub = NewObject<UTopic>(UTopic::StaticClass());
		FString ASGClientStatusTopic = FString::Printf(TEXT("/%s/status"), *AutoSceneGenClientName);
		ASGClientStatusSub->Init(ROSInst->ROSIntegrationCore, ASGClientStatusTopic, TEXT("auto_scene_gen_msgs/StatusCode"));
		ASGClientStatusSub->Subscribe(std::bind(&AAutoSceneGenWorker::ASGClientStatusCB, this, std::placeholders::_1));
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS subscriber: %s"), *ASGClientStatusTopic);

		// ASG worker status pub
		WorkerStatusPub = NewObject<UTopic>(UTopic::StaticClass());
		FString WorkerStatusTopic = FString::Printf(TEXT("/asg_worker%i/status"), WorkerID);
		WorkerStatusPub->Init(ROSInst->ROSIntegrationCore, WorkerStatusTopic, TEXT("auto_scene_gen_msgs/StatusCode"));
		WorkerStatusPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS publisher: %s"), *WorkerStatusTopic);

		// Vehicle destination pub
		VehicleDestinationPub = NewObject<UTopic>(UTopic::StaticClass());
		FString VehicleDestinationTopic = FString::Printf(TEXT("/asg_worker%i/nav/destination"), WorkerID);
		VehicleDestinationPub->Init(ROSInst->ROSIntegrationCore, VehicleDestinationTopic, TEXT("geometry_msgs/Pose"));
		VehicleDestinationPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS publisher: %s"), *VehicleDestinationTopic);
		
		// AnalyzeScenario client
		AnalyzeScenarioClient = NewObject<UService>(UService::StaticClass());
		FString AnalyzeScenarioServiceName = FString::Printf(TEXT("/%s/services/analyze_scenario"), *AutoSceneGenClientName);
		AnalyzeScenarioClient->Init(ROSInst->ROSIntegrationCore, AnalyzeScenarioServiceName, TEXT("auto_scene_gen_msgs/AnalyzeScenario"));
		UE_LOG(LogASG, Display, TEXT("Registered ASG worker analyze scenario ROS client to: %s"), *AnalyzeScenarioServiceName);

		// RunScenario service
		RunScenarioService = NewObject<UService>(UService::StaticClass());
		FString RunScenarioServiceName = FString::Printf(TEXT("/asg_worker%i/services/run_scenario"), WorkerID);
		RunScenarioService->Init(ROSInst->ROSIntegrationCore, RunScenarioServiceName, TEXT("auto_scene_gen_msgs/RunScenario"));
		RunScenarioService->Advertise(std::bind(&AAutoSceneGenWorker::RunScenarioServiceCB, this, std::placeholders::_1, std::placeholders::_2), false);
		UE_LOG(LogASG, Display, TEXT("Registered ASG worker run scenario ROS service: %s"), *RunScenarioServiceName);
	}
	else
	{
		UE_LOG(LogASG, Display, TEXT("ASG worker will create scenes randomly (for debugging purposes)."));
	}

	// Calling this here forces the engine to render the debug SSAs before the tick function
	RandomizeDebugStructuralSceneActors();
}

void AAutoSceneGenWorker::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);

	DebugSSASubclasses.Empty();
	SSAMaintainerMap.Empty();
	RequestedSSAArray.Empty();

	if (ROSInst)
	{
		// Publish offline status. Publish numerous messages with the hope that 1 or 2 are received.
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::StatusCode> StatusMsg(new ROSMessages::auto_scene_gen_msgs::StatusCode(ROSMessages::auto_scene_gen_msgs::StatusCode::OFFLINE));
		for (int32 I = 0; I < 10; I++)
		{
			WorkerStatusPub->Publish(StatusMsg);
			FPlatformProcess::Sleep(0.01f); // Brief pause helps ensure the messages are received
		}
		WorkerStatusPub->Unadvertise();

		ASGClientStatusSub->Unsubscribe();
		VehicleDestinationPub->Unadvertise();
		RunScenarioService->Unadvertise();
	}
}

// Called every frame
void AAutoSceneGenWorker::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (ASGVehicle && bForceVehicleReset)
	{
		UE_LOG(LogASG, Warning, TEXT("Forcing vehicle reset"));
		ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, true);
		bForceVehicleReset = false;
	}
	
	if (!bProcessedScenarioRequest)
	{
		ProcessRunScenarioRequest();
	}
	
	// This forces the worker to wait one tick for the frame to render before publishing its status
	if (ASGVehicle && !bReadyToTick)
	{
		bReadyToTick = true;
		ASGVehicle->SetWorldIsReadyFlag(true);
		return;
	}

	if (bProcessedScenarioRequest && !bWaitingForScenarioRequest)
	{
		CheckIfVehicleCrashed();
		CheckIfVehicleFlipped();
		CheckGoalLocation();
	}

	if (ROSInst)
	{
		// Publish ASG worker status
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::StatusCode> StatusMsg(new ROSMessages::auto_scene_gen_msgs::StatusCode(WorkerStatus));
		WorkerStatusPub->Publish(StatusMsg);
		
		// Publish vehicle destination info
		TSharedPtr<ROSMessages::geometry_msgs::Pose> DestMsg(new ROSMessages::geometry_msgs::Pose());
		ROSMessages::geometry_msgs::Point Location(VehicleGoalLocation/100.f); // Put into [m]
		Location.y *= -1;
		DestMsg->position = Location;
		ROSMessages::geometry_msgs::Quaternion Quaternion = ROSMessages::geometry_msgs::Quaternion(0,0,0,1);
		Quaternion.x *= -1;
		Quaternion.z *= -1;
		DestMsg->orientation = Quaternion;
		VehicleDestinationPub->Publish(DestMsg);

		// Occasionaly, for some unknown reason, resetting the vehicle (via teleportation) can throw the vehicle out of the bounds of the game.
		// To account for this phenomena, if the vehicle is ever below the ground plane, then we reset the vehicle and try again.
		// NOTE: Make sure to uncheck EnableWorldBoundsCheck in world settings, since we do not want UE4 deleting the vehicle from the game
		if (ASGVehicle && ASGVehicle->GetActorLocation().Z <= GroundPlaneZHeight -100.)
		{
			UE_LOG(LogASG, Warning, TEXT("Vehicle was thrown out of bounds or went off the landscape. Resetting so we can try again."));
			bProcessedScenarioRequest = false; // This will reset the run for us
			WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
			return;
		}
	}
}

uint8 AAutoSceneGenWorker::GetWorkerID() const
{
	return WorkerID;
}

void AAutoSceneGenWorker::InitDebugStructuralSceneActors()  // Remove?
{
	TArray<bool> Visibilities;
	TArray<bool> CastShadows;
	TArray<FVector> Locations;
	TArray<FRotator> Rotations;
	TArray<float> Scales;

	// For the debug SSAs, we use the same number of instances of each subclass
	for (int32 i = 0; i < DebugNumSSAInstances; i++)
	{
		Visibilities.Emplace(true);
		CastShadows.Emplace(bDebugSSACastShadow);
		Locations.Emplace(FVector(0,0,GroundPlaneZHeight));
		Rotations.Emplace(FRotator(0,0,0));
		Scales.Emplace(1.f);
	}

	for (TSubclassOf<AStructuralSceneActor> Subclass : DebugSSASubclasses)
	{
		SSAMaintainerMap[Subclass->GetPathName()]->UpdateAttributes(Visibilities, CastShadows, Locations, Rotations, Scales);
	}
	
	bSSAInit = true;
}

void AAutoSceneGenWorker::RandomizeDebugStructuralSceneActors()
{
	for (TSubclassOf<AStructuralSceneActor> Subclass : DebugSSASubclasses)
	{
		TArray<bool> Visibilities;
		TArray<bool> CastShadows;
		TArray<FVector> Locations;
		TArray<FRotator> Rotations;
		TArray<float> Scales;

		// For the debug SSAs, we use the same number of instances of each subclass
		for (int32 i = 0; i < DebugNumSSAInstances; i++)
		{
			CastShadows.Emplace(bDebugSSACastShadow);
			FVector Location = FVector(FMath::RandRange(0.f, 1.f) * LandscapeSize.X, -FMath::RandRange(0.f, 1.f) * LandscapeSize.Y, GroundPlaneZHeight);
			Locations.Emplace(Location);
			Rotations.Emplace(FRotator(0, FMath::RandRange(0.f, 1.f) * 360.f, 0));
			Scales.Emplace(1.f);

			// Determine visibility based on SSA placement
			FVector Loc = Location;
			Loc.Z = VehicleStartLocation.Z;
			if ((Loc - VehicleStartLocation).Size() <= DebugSafetyRadius || (Loc - VehicleGoalLocation).Size() <= DebugSafetyRadius)
			{
				Visibilities.Emplace(false);
			}
			else
			{
				Visibilities.Emplace(true);
			}
		}
		
		SSAMaintainerMap[Subclass->GetPathName()]->UpdateAttributes(Visibilities, CastShadows, Locations, Rotations, Scales);
	}
}

void AAutoSceneGenWorker::ProcessRunScenarioRequest() 
{
	if (!ROSInst || !ASGVehicle || !bASGClientOnline || bProcessedScenarioRequest) return;

	ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation);

	// Store requested SSA subclasses in array of TSubclassOf<AStructuralSceneActor> so we can access them later
	TArray<TSubclassOf<AStructuralSceneActor>> RequestedSSASubclasses;

	// Verify path_names to SSA subclasses (can only be done in game thread)
	for (ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout Layout : RequestedSSAArray)
	{
		UBlueprintGeneratedClass* CastBP = LoadObject<UBlueprintGeneratedClass>(nullptr, *Layout.path_name, nullptr, LOAD_None, nullptr);

		if (CastBP)
		{
			if (CastBP->IsChildOf(AStructuralSceneActor::StaticClass()))
			{
				RequestedSSASubclasses.Add(CastBP);
			}
			else
			{
				UE_LOG(LogASG, Error, TEXT("Failed to process RunScenarioRequest: UObject with path_name %s is not a child of AStructuralSceneActor"), *Layout.path_name);
				return;
			}
		}
		else
		{
			UE_LOG(LogASG, Error, TEXT("Failed to process RunScenarioRequest: Could not cast UObject with path_name %s to UBlueprintGeneratedClass"), *Layout.path_name);
			return;
		}
	}

	// Add SSA subclasses to maintainer map and update attributes
	for (TSubclassOf<AStructuralSceneActor> Subclass : RequestedSSASubclasses)
	{
		// It is more efficient to only add a new subclass to the TMap if the key does not already exist, otherwise the old objects will not get gc'd immediately.
		if (!SSAMaintainerMap.Contains(Subclass->GetPathName()))
		{
			SSAMaintainerMap.Add(Subclass->GetPathName(), NewObject<UStructuralSceneActorMaintainer>(UStructuralSceneActorMaintainer::StaticClass()));
			SSAMaintainerMap[Subclass->GetPathName()]->Init(GetWorld(), Subclass);
		}

		// Find corresponding element in SSA array
		for (ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout Layout : RequestedSSAArray)
		{
			if (Subclass->GetPathName().Equals(Layout.path_name))
			{
				TArray<FVector> Locations;
				TArray<FRotator> Rotations;

				for (int32 i = 0; i < Layout.num_instances; i++)
				{
					Locations.Emplace(FVector(Layout.x_positions[i], Layout.y_positions[i], GroundPlaneZHeight));
					Rotations.Emplace(FRotator(0, Layout.yaw_angles[i], 0));
				}
				
				SSAMaintainerMap[Subclass->GetPathName()]->UpdateAttributes(Layout.visibilities, Layout.cast_shadows, Locations, Rotations, Layout.scale_factors);
				break;
			}
		}
	}
	
	bReadyToTick = false;
	bProcessedScenarioRequest = true;
	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_RUNNING;
	UE_LOG(LogASG, Display, TEXT("Processed scenario description %i"), ScenarioNumber);
}

bool AAutoSceneGenWorker::CheckIfVehicleFlipped()
{
	if (!ASGVehicle || !ASGVehicle->IsEnabled()) return false;

	if (FMath::Abs(ASGVehicle->GetActorRotation().Euler().X) > MaxVehicleRoll)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle roll %i degrees exceeds %i degree maximum. Resetting vehicle."), ASGVehicle->GetActorRotation().Euler().X, MaxVehicleRoll);
		ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, true);
		ASGVehicle->SetWorldIsReadyFlag(true);
		return true;
	}
	if ( FMath::Abs(ASGVehicle->GetActorRotation().Euler().Y) > MaxVehiclePitch)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle pitch %i degrees exceeds %i degree maximum. Resetting vehicle."), ASGVehicle->GetActorRotation().Euler().Y, MaxVehiclePitch);
		ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, true);
		ASGVehicle->SetWorldIsReadyFlag(true);
		return true;
	}
	return false;
}

bool AAutoSceneGenWorker::CheckIfVehicleCrashed()
{
	if (ROSInst && ASGVehicle && ASGVehicle->GetNumStructuralSceneActorsHit() > 0)
	{
		UE_LOG(LogASG, Display, TEXT("Vehicle has crashed. Vehicle has failed."));
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest> Req(new ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest());
		ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, Req->vehicle_trajectory);
		
		bWaitingForScenarioRequest = true;
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
		Req->worker_id = WorkerID;
		Req->scenario_number = ScenarioNumber;
		Req->crashed = true;
		Req->succeeded = false;

		UE_LOG(LogASG, Warning, TEXT("Submitting AnalyzeScenario request %i"), ScenarioNumber);
		AnalyzeScenarioClient->CallService(Req, std::bind(&AAutoSceneGenWorker::AnalyzeScenarioResponseCB, this, std::placeholders::_1));
		return true;
	}
	return false;
}

bool AAutoSceneGenWorker::CheckGoalLocation() 
{
	if (!ASGVehicle) return false;

	FVector DistanceToGoal = ASGVehicle->GetActorLocation() - VehicleGoalLocation;
	DistanceToGoal.Z = 0;
	if (DistanceToGoal.Size() <= GoalRadius)
	{
		UE_LOG(LogASG, Display, TEXT("Vehicle reached the goal radius. Vehicle has succeeded."));
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest> Req(new ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest());
		ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, Req->vehicle_trajectory);

		if (ROSInst && !bWaitingForScenarioRequest /*&& bASGClientOnline*/)
		{
			bWaitingForScenarioRequest = true;
			WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;

			Req->worker_id = WorkerID;
			Req->scenario_number = ScenarioNumber;
			Req->crashed = false;
			Req->succeeded = true;

			UE_LOG(LogASG, Display, TEXT("Submitting AnalyzeScenario request %i"), ScenarioNumber);
			AnalyzeScenarioClient->CallService(Req, std::bind(&AAutoSceneGenWorker::AnalyzeScenarioResponseCB, this, std::placeholders::_1));
			return true;
		}
		if (!ROSInst)
		{
			RandomizeDebugStructuralSceneActors();
			UE_LOG(LogASG, Display, TEXT("No ROSIntegration game instance. Generating new random scene."));
		}
	}
	return false;
}

void AAutoSceneGenWorker::ASGClientStatusCB(TSharedPtr<FROSBaseMsg> Msg) 
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::StatusCode>(Msg);
	if (!CastMsg)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast msg to auto_scene_gen_msgs/StatusCode"));
		return;
	}

	if (!bASGClientOnline && CastMsg->status == ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_RUNNING)
	{
		UE_LOG(LogASG, Display, TEXT("ASG client back online"));
	}

	// We assume the ASG client is configured to send an offline signal whenever it gets shutdown
	if (bASGClientOnline && CastMsg->status != ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_RUNNING)
	{
		bForceVehicleReset = true; // This will reset the run for us
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
		UE_LOG(LogASG, Warning, TEXT("ASG client went offline"));
	}
	
	bASGClientOnline = CastMsg->status == ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_RUNNING;
}

void AAutoSceneGenWorker::AnalyzeScenarioResponseCB(TSharedPtr<FROSBaseServiceResponse> Response) 
{
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioResponse>(Response);
	if (!CastResponse)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast msg to auto_scene_gen_msgs/AnalyzeScenarioResponse"));
		return;
	}
	UE_LOG(LogASG, Display, TEXT("AnalyzeScenario request received: %s"), (CastResponse->received ? *FString("True") : *FString("False")));
}

void AAutoSceneGenWorker::RunScenarioServiceCB(TSharedPtr<FROSBaseServiceRequest> Request, TSharedPtr<FROSBaseServiceResponse> Response) 
{
	auto CastRequest = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::FRunScenarioRequest>(Request);
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::FRunScenarioResponse>(Response);
	if (!CastRequest)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast Request to auto_scene_gen_msgs/RunScenarioRequest"));
		return;
	}
	if (!CastResponse)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast Response to auto_scene_gen_msgs/RunScenarioResponse"));
		return;
	}

	UE_LOG(LogASG, Display, TEXT("Received RunScenario request %i"), CastRequest->scenario_number);

	ScenarioNumber = CastRequest->scenario_number;
	VehicleStartLocation = FVector(CastRequest->vehicle_start_location.x, CastRequest->vehicle_start_location.y, GroundPlaneZHeight);
	VehicleStartRotation = FRotator(0, CastRequest->vehicle_start_yaw, 0);
	VehicleGoalLocation = FVector(CastRequest->vehicle_goal_location.x, CastRequest->vehicle_goal_location.y, GroundPlaneZHeight);
	GoalRadius = CastRequest->goal_radius;

	RequestedSSAArray.Empty();
	RequestedSSAArray = CastRequest->ssa_array;
	if (RequestedSSAArray.Num() == 0)
	{
		UE_LOG(LogASG, Error, TEXT("RunScenario request field 'ssa_array' is empty"));
	}

	bWaitingForScenarioRequest = false;
	bProcessedScenarioRequest = false;
	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
	UE_LOG(LogASG, Display, TEXT("Saved scenario description %i"), ScenarioNumber);

	// Fill in response
	CastResponse->received = true;
}
