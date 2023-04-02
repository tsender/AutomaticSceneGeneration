// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/AutoSceneGenWorker.h"
#include "Actors/AutoSceneGenLandscape.h"
#include "Objects/StructuralSceneActorMaintainer.h"
#include "Actors/StructuralSceneActor.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "AutoSceneGenLogging.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/DirectionalLight.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/KismetMathLibrary.h"

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
#include "auto_scene_gen_msgs/srv/WorkerIssueNotificationRequest.h"
#include "auto_scene_gen_msgs/srv/WorkerIssueNotificationResponse.h"

// Sets default values
AAutoSceneGenWorker::AAutoSceneGenWorker()
{
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AAutoSceneGenWorker::BeginPlay()
{
	Super::BeginPlay();

	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::OFFLINE; // Only becomes online when rosbridge connection is healthy
	ScenarioNumber = 0;
	bASGClientOnline = false;
	bROSBridgeHealthy = false;
	bROSBridgeConnectionInterrupted = false;
	bForceVehicleReset = false;
	bWaitingForScenarioRequest = true; // false
	bProcessedScenarioRequest = true;
	bReadyToTick = false;

	// Find ground plane actor
	// TArray<AActor*> TempArray;
	// UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), TempArray);
	// if (!TempArray.Num())
	// {
	// 	UE_LOG(LogASG, Error, TEXT("The ground plane must be a StaticMeshActors with tag 'ground_plane', but could not find any StaticMeshActors in the scene."));
	// 	return;
	// }
	// for (AActor* Actor: TempArray)
	// {
	// 	if (Actor->ActorHasTag("ground_plane"))
	// 	{
	// 		FVector Origin;
	// 		FVector BoxExtent;
	// 		Actor->GetActorBounds(true, Origin, BoxExtent);
	// 		GroundPlaneZHeight = Origin.Z + BoxExtent.Z;
	// 		// LandscapeSize = 2*BoxExtent;
	// 		break;
	// 	}
	// }

	// Get landscape actor
	TArray<AActor*> TempArray;
	TempArray.Empty();
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAutoSceneGenLandscape::StaticClass(), TempArray);
	if (!TempArray.Num())
	{
		UE_LOG(LogASG, Error, TEXT("There must exist an AutoSceneGenLandcape actor in the world."));
		return;
	}
	ASGLandscape = Cast<AAutoSceneGenLandscape>(TempArray[0]);
	if (!ASGLandscape)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast AutoSceneGenLandcape actor."));
		return;
	}
	if (!LandscapeMaterial)
	{
		UE_LOG(LogASG, Error, TEXT("Landscape material is nullptr. Cannot create landscape without material."));
		return;
	}
	ASGLandscape->CreateBaseMesh(FVector(0.,-LandscapeSize,0.), LandscapeSize, DebugLandscapeSubdivisions, LandscapeBorder);
	ASGLandscape->SetMaterial(LandscapeMaterial);
	// ASGLandscape->LandscapeEditSculptCircularPatch(FVector(LandscapeSize/2., LandscapeSize/2, 0.), LandscapeSize/4., 20.*100, true, 0, ELandscapeFalloff::Smooth);
	// ASGLandscape->LandscapeEditSculptRamp(FVector(100.*100, 100.*100, 0.), FVector(150.*100, 150*100., 10.*100), 50.*100, false, false, 0.7, ELandscapeFalloff::Smooth);
	// ASGLandscape->LandscapeEditSculptRamp(FVector(100.*100, 200.*100, 0.*100.), FVector(100.*100, 300*100., 10.*100), 50.*100, true, false, 0.5, ELandscapeFalloff::Smooth);
	// ASGLandscape->LandscapeEditSculptCircularPatch(FVector(100.*100, 250.*100, 0.), 25.*100, -10.*100, true, 0., ELandscapeFalloff::Smooth);
	// ASGLandscape->LandscapeEditSculptRamp(FVector(200.*100, 200.*100, -40.*100.), FVector(300.*100, 300*100., 10.*100), 50.*100, true, false, 0.5, ELandscapeFalloff::Smooth);
	// ASGLandscape->LandscapeEditSculptRamp(FVector(100.*100, 250.*100, 0.*100.), FVector(250.*100, 250*100., 10.*100), 60.*100, true, true, 0.2, ELandscapeFalloff::Smooth);
	// ASGLandscape->LandscapeEditSculptRamp(FVector(350.*100, 350.*100, 10.*100.), FVector(250.*100, 450*100., 10.*100), 60.*100, true, true, 0.0, ELandscapeFalloff::Smooth);
	// ASGLandscape->PostSculptUpdate();

	// Get light source actor (for sunlight)
	TempArray.Empty();
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ADirectionalLight::StaticClass(), TempArray);
	if (!TempArray.Num())
	{
		UE_LOG(LogASG, Error, TEXT("There must be a DirectionalLight actor in the world to act as sunlight."));
		return;
	}
	LightSource = Cast<ADirectionalLight>(TempArray[0]);
	if (!LightSource)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast DirectionalLight actor."));
		return;
	}

	// Find ASG vehicle
	VehicleStartRotation = FRotator(0.f, VehicleStartYaw, 0.f);
	TempArray.Empty();
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAutoSceneGenVehicle::StaticClass(), TempArray);
	if (!TempArray.Num())
	{
		UE_LOG(LogASG, Error, TEXT("There must be an AutoSceneGenVehicle actor (either from first player controller or manually added)."));
		return;
	}
	// ASGVehicle = Cast<AAutoSceneGenVehicle>(GetWorld()->GetFirstPlayerController()->GetPawn());
	ASGVehicle = Cast<AAutoSceneGenVehicle>(TempArray[0]);
	if (!ASGVehicle)
	{
		// UE_LOG(LogASG, Error, TEXT("Could not get AutoSceneGenVehicle from first player controller."));
		UE_LOG(LogASG, Error, TEXT("Failed to cast AutoSceneGenVehicle."));
		return;
	}
	SetVehicleStartZLocation();
	ASGVehicle->SetDefaultResetInfo(VehicleStartLocation, VehicleStartRotation);

	// Add debug structural scene actors to SSA maintainer map
	for (TSubclassOf<AStructuralSceneActor> Subclass : DebugSSASubclasses)
	{
		SSAMaintainerMap.Add(Subclass->GetPathName(), NewObject<UStructuralSceneActorMaintainer>(UStructuralSceneActorMaintainer::StaticClass()));
		SSAMaintainerMap[Subclass->GetPathName()]->Init(GetWorld(), Subclass);
	}

	ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	if (ROSInst)
	{
		ROSInst->OnROSConnectionStatus.AddUObject(this, &AAutoSceneGenWorker::OnROSConnectionStatus);
		
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
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS client: %s"), *AnalyzeScenarioServiceName);

		// WorkerIssueNotification client
		WorkerIssueNotificationClient = NewObject<UService>(UService::StaticClass());
		FString WorkerIssueNotificationServiceName = FString::Printf(TEXT("/%s/services/worker_issue_notification"), *AutoSceneGenClientName);
		WorkerIssueNotificationClient->Init(ROSInst->ROSIntegrationCore, WorkerIssueNotificationServiceName, TEXT("auto_scene_gen_msgs/WorkerIssueNotification"));
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS client: %s"), *WorkerIssueNotificationServiceName);

		// RunScenario service
		RunScenarioService = NewObject<UService>(UService::StaticClass());
		FString RunScenarioServiceName = FString::Printf(TEXT("/asg_worker%i/services/run_scenario"), WorkerID);
		RunScenarioService->Init(ROSInst->ROSIntegrationCore, RunScenarioServiceName, TEXT("auto_scene_gen_msgs/RunScenario"));
		RunScenarioService->Advertise(std::bind(&AAutoSceneGenWorker::RunScenarioServiceCB, this, std::placeholders::_1, std::placeholders::_2), false);
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS service: %s"), *RunScenarioServiceName);
	}
	else
	{
		UE_LOG(LogASG, Display, TEXT("ASG worker will create scenes randomly (for debugging purposes)."));
	}

	// Calling this here forces the engine to render the debug SSAs before the tick function
	RandomizeDebugStructuralSceneActors();

	FTimerHandle TimerHandle;
	GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &AAutoSceneGenWorker::PublishStatus, 0.05, true);
}

void AAutoSceneGenWorker::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);

	DebugSSASubclasses.Empty();
	SSAMaintainerMap.Empty();
	SceneDescription.ssa_array.Empty();

	if (ROSInst)
	{
		// Publish offline status. Publish numerous messages with the hope that 1 or 2 are received.
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::StatusCode> StatusMsg(new ROSMessages::auto_scene_gen_msgs::StatusCode(ROSMessages::auto_scene_gen_msgs::StatusCode::OFFLINE));
		for (int32 I = 0; I < 10; I++)
		{
			WorkerStatusPub->Publish(StatusMsg);
			FPlatformProcess::Sleep(0.01f); // Brief pause helps ensure the messages are received
		}
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
	
	// Since the renderer is one frame behind the game thread, this boolean forces us to wait an extra tick
	// to ensure the vehicle is enabled when the scene is fully rendered.
	if (ASGVehicle && !bReadyToTick)
	{
		bReadyToTick = true;
		ASGVehicle->SetWorldIsReadyFlag(true);
		return;
	}

	if (bProcessedScenarioRequest && !bWaitingForScenarioRequest)
	{
		CheckForVehicleReset();
		CheckGoalLocation();
	}

	if (ROSInst)
	{
		if (bROSBridgeConnectionInterrupted && bASGClientOnline)
		{
			SendWorkerIssueNotification(ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest::ISSUE_ROSBRIDGE_INTERRUPTED, FString(""));
			bROSBridgeConnectionInterrupted = false;
		}
		
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
		// If the vehicle is ever below the landscape's lowest point or is outside the landscape XY bounds, then we reset the vehicle and try again.
		// NOTE: Make sure to uncheck EnableWorldBoundsCheck in world settings, since we do not want UE4 deleting the vehicle from the game
		FBox LandscapeBoundingBox = ASGLandscape->GetBoundingBox();
		if (ASGVehicle && 
			(ASGVehicle->GetActorLocation().Z <= LandscapeBoundingBox.Min.Z + ASGLandscape->GetActorLocation().Z - 100.
			|| ASGVehicle->GetActorLocation().Z >= LandscapeBoundingBox.Max.Z + ASGLandscape->GetActorLocation().Z + 1000.
			|| !LandscapeBoundingBox.IsInsideXY(ASGVehicle->GetActorLocation() - ASGLandscape->GetActorLocation())) )
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

void AAutoSceneGenWorker::RandomizeDebugStructuralSceneActors()
{
	TMap<FString, TArray<FHitResult>> SurfaceData;
	
	// Make SSAs invisible initially so we can get landscape surface data
	for (TSubclassOf<AStructuralSceneActor> Subclass : DebugSSASubclasses)
	{
		TArray<bool> Visibilities;
		TArray<bool> CastShadows;
		TArray<FVector> Locations;
		TArray<FRotator> Rotations;
		TArray<float> Scales;

		Visibilities.Init(false, DebugNumSSAInstances);
		CastShadows.Init(bDebugSSACastShadow, DebugNumSSAInstances);
		Rotations.Init(FRotator(0.), DebugNumSSAInstances);
		Scales.Init(1., DebugNumSSAInstances);

		// For the debug SSAs, we use the same number of instances of each subclass
		TArray<FHitResult> Hits;
		for (int32 i = 0; i < DebugNumSSAInstances; i++)
		{
			FVector Location = FVector(FMath::RandRange(0.f, 1.f) * LandscapeSize, -FMath::RandRange(0.f, 1.f) * LandscapeSize, 0.);
			Locations.Emplace(Location);
			FHitResult Hit;
			TArray<AActor*> ActorsToIgnore;
			ASGLandscape->GetLandscapeSurfaceData(Location - ASGLandscape->GetActorLocation(), ActorsToIgnore, Hit);
			Hits.Emplace(Hit);
		}
		
		SurfaceData.Add(Subclass->GetPathName(), Hits);
		SSAMaintainerMap[Subclass->GetPathName()]->UpdateAttributes(Visibilities, CastShadows, Locations, Rotations, Scales);
	}
	
	// Now set remaining attributes correctly
	for (TSubclassOf<AStructuralSceneActor> Subclass : DebugSSASubclasses)
	{
		for (int32 i = 0; i < DebugNumSSAInstances; i++)
		{
			AStructuralSceneActor* Actor =  SSAMaintainerMap[Subclass->GetPathName()]->GetActor(i);
			FVector Location = Actor->GetActorLocation();
			Location.Z = SurfaceData[Subclass->GetPathName()][i].ImpactPoint.Z;
			Actor->SetActorLocation(Location);
			Actor->SetActorRotation(UKismetMathLibrary::MakeRotFromZ(SurfaceData[Subclass->GetPathName()][i].ImpactNormal));

			// Determine visibility based on SSA placement
			if ((Location - VehicleStartLocation).Size2D() <= DebugSafetyRadius || (Location - VehicleGoalLocation).Size2D() <= DebugSafetyRadius)
				Actor->SetActive(false);
			else
				Actor->SetActive(true);
		}
	}
}

void AAutoSceneGenWorker::SetVehicleStartZLocation()
{
	FVector Origin;
	FVector BoxExtent;
	ASGVehicle->GetActorBounds(true, Origin, BoxExtent);

	FVector ForwardVec = ASGVehicle->GetActorForwardVector();
	FVector RightVec = ASGVehicle->GetActorRightVector();

	// Find max Z between current location and 4 points on the edges of the bounding box
	TArray<AActor*> TempArray;
	float MaxZ = ASGLandscape->GetLandscapeElevation(VehicleStartLocation - ASGLandscape->GetActorLocation(), TempArray);
	MaxZ = FMath::Max<float>(MaxZ, ASGLandscape->GetLandscapeElevation(VehicleStartLocation + ForwardVec*BoxExtent.X - ASGLandscape->GetActorLocation(), TempArray));
	MaxZ = FMath::Max<float>(MaxZ, ASGLandscape->GetLandscapeElevation(VehicleStartLocation - ForwardVec*BoxExtent.X - ASGLandscape->GetActorLocation(), TempArray));
	MaxZ = FMath::Max<float>(MaxZ, ASGLandscape->GetLandscapeElevation(VehicleStartLocation + RightVec*BoxExtent.Y - ASGLandscape->GetActorLocation(), TempArray));
	MaxZ = FMath::Max<float>(MaxZ, ASGLandscape->GetLandscapeElevation(VehicleStartLocation - RightVec*BoxExtent.Y - ASGLandscape->GetActorLocation(), TempArray));

	VehicleStartLocation.Z = MaxZ + 100.; // Add 1 meter to max Z for safety
}

void AAutoSceneGenWorker::PublishStatus()
{
	if (ROSInst)
	{
		// Publish ASG worker status
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::StatusCode> StatusMsg(new ROSMessages::auto_scene_gen_msgs::StatusCode(WorkerStatus));
		WorkerStatusPub->Publish(StatusMsg);
	}
}

void AAutoSceneGenWorker::SendWorkerIssueNotification(uint8 IssueID, FString ErrorMessage)
{
	if (ROSInst && bASGClientOnline)
	{
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest> Req(new ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest());
		Req->worker_id = WorkerID;
		Req->issue_id = IssueID;
		Req->message = ErrorMessage;
		UE_LOG(LogASG, Display, TEXT("Sending WorkerIssueNotification request to ASG client."));
		WorkerIssueNotificationClient->CallService(Req, std::bind(&AAutoSceneGenWorker::WorkerIssueNotificationResponseCB, this, std::placeholders::_1));
	}
}

void AAutoSceneGenWorker::ProcessRunScenarioRequest() 
{
	if (!ROSInst || !bASGClientOnline || !ASGVehicle || !LightSource || bProcessedScenarioRequest) return;

	// TODO: If there are any errors parsing the new scenario request, send a message to the AutoSceneGenClient with the error message

	// TODO: Add code to modify landscape from description
	ASGLandscape->CreateBaseMesh(FVector(0.,-SceneDescription.landscape.nominal_size,0.), 
								SceneDescription.landscape.nominal_size, 
								SceneDescription.landscape.subdivisions, 
								SceneDescription.landscape.border);

	SetVehicleStartZLocation();
	ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation);
	LightSource->SetActorRotation(FRotator(-SceneDescription.sunlight_inclination, SceneDescription.sunlight_yaw_angle, 0.));

	// Store requested SSA subclasses in array of TSubclassOf<AStructuralSceneActor> so we can access them later
	TArray<TSubclassOf<AStructuralSceneActor>> RequestedSSASubclasses;

	// Verify path_names to SSA subclasses (can only be done in game thread)
	for (ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout Layout : SceneDescription.ssa_array)
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
				FString ErrorMessage = FString::Printf(TEXT("Failed to process RunScenarioRequest: UObject with path_name %s is not a child of AStructuralSceneActor"), *Layout.path_name);
				UE_LOG(LogASG, Error, TEXT("%s"), *ErrorMessage);
				SendWorkerIssueNotification(ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest::ISSUE_PROBLEM_CREATING_SCENE, ErrorMessage);
				return;
			}
		}
		else
		{
			FString ErrorMessage = FString::Printf(TEXT("Failed to process RunScenarioRequest: Could not cast UObject with path_name %s to UBlueprintGeneratedClass"), *Layout.path_name);
			UE_LOG(LogASG, Error, TEXT("%s"), *ErrorMessage);
			SendWorkerIssueNotification(ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest::ISSUE_PROBLEM_CREATING_SCENE, ErrorMessage);
			return;
		}
	}

	// Make everything invisible so we can easily get landscape surface data
	for (TPair<FString, UStructuralSceneActorMaintainer*> Elem : SSAMaintainerMap)
		Elem.Value->SetAllActorsInvisible();

	// Make sure we have the correct number of SSAs per requested subclass
	TMap<FString, TArray<FHitResult>> SurfaceData;
	for (TSubclassOf<AStructuralSceneActor> Subclass : RequestedSSASubclasses)
	{
		// It is more efficient to only add a new subclass to the TMap if the key does not already exist, otherwise the old objects will not get gc'd immediately.
		if (!SSAMaintainerMap.Contains(Subclass->GetPathName()))
		{
			SSAMaintainerMap.Add(Subclass->GetPathName(), NewObject<UStructuralSceneActorMaintainer>(UStructuralSceneActorMaintainer::StaticClass()));
			SSAMaintainerMap[Subclass->GetPathName()]->Init(GetWorld(), Subclass);
		}

		for (ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout Layout : SceneDescription.ssa_array)
		{
			if (Subclass->GetPathName().Equals(Layout.path_name))
			{
				SSAMaintainerMap[Subclass->GetPathName()]->SetNumActors(Layout.num_instances, false);

				// Get surface data
				TArray<FHitResult> Hits;
				for (int32 i = 0; i < Layout.num_instances; i++)
				{
					FVector Location = FVector(Layout.x[i], Layout.y[i], 0.);
					FHitResult Hit;
					TArray<AActor*> ActorsToIgnore;
					ASGLandscape->GetLandscapeSurfaceData(Location - ASGLandscape->GetActorLocation(), ActorsToIgnore, Hit);
					Hits.Emplace(Hit);
				}
				SurfaceData.Add(Subclass->GetPathName(), Hits);
				break;
			}
		}
	}

	// Set attributes for all requested SSA subclasses
	for (TSubclassOf<AStructuralSceneActor> Subclass : RequestedSSASubclasses)
	{
		// Find corresponding element in SSA array
		for (ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout Layout : SceneDescription.ssa_array)
		{
			if (Subclass->GetPathName().Equals(Layout.path_name))
			{
				TArray<FVector> Locations;
				TArray<FRotator> Rotations;

				for (int32 i = 0; i < Layout.num_instances; i++)
				{
					AStructuralSceneActor* Actor =  SSAMaintainerMap[Subclass->GetPathName()]->GetActor(i);
					Actor->SetActorLocation(FVector(Layout.x[i], Layout.y[i], SurfaceData[Subclass->GetPathName()][i].ImpactPoint.Z));

					if (!Actor->ActorHasTag(TEXT("tree")))
						Actor->SetActorRotation(UKismetMathLibrary::MakeRotFromZ(SurfaceData[Subclass->GetPathName()][i].ImpactNormal));
					Actor->AddActorLocalRotation(FRotator(0., Layout.yaw[i], 0.));

					Actor->SetActive(Layout.visible[i]);
					Actor->SetCastShadow(Layout.cast_shadow[i]);
					Actor->SetScale(Layout.scale[i]);
				}
				break;
			}
		}
	}
	
	bReadyToTick = false;
	bProcessedScenarioRequest = true;
	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_RUNNING;
	UE_LOG(LogASG, Display, TEXT("Processed scenario description %i. ASG worker is ONLINE_AND_RUNNING."), ScenarioNumber);
}

void AAutoSceneGenWorker::ResetVehicleAndSendAnalyzeScenarioRequest(uint8 TerminationReason)
{
	TSharedPtr<ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest> Req(new ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest());
	ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, Req->vehicle_trajectory);
	
	bWaitingForScenarioRequest = true;
	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
	Req->worker_id = WorkerID;
	Req->scenario_number = ScenarioNumber;
	Req->termination_reason = TerminationReason;

	FROSTime start_time = Req->vehicle_trajectory[0].header.time;
	FROSTime end_time = Req->vehicle_trajectory[Req->vehicle_trajectory.Num()-1].header.time;
	Req->vehicle_sim_time = FROSTime::GetTimeDelta(start_time, end_time);

	UE_LOG(LogASG, Display, TEXT("Computed vehicle sim time: %f"), Req->vehicle_sim_time);
	UE_LOG(LogASG, Warning, TEXT("Submitting AnalyzeScenario request %i. ASG worker is ONLINE_AND_READY."), ScenarioNumber);
	AnalyzeScenarioClient->CallService(Req, std::bind(&AAutoSceneGenWorker::AnalyzeScenarioResponseCB, this, std::placeholders::_1));
}

bool AAutoSceneGenWorker::CheckForVehicleReset()
{
	if (!ASGVehicle || !ASGVehicle->IsEnabled()) return false;

	// Did vehicle crash?
	if (ROSInst && !bAllowCollisions && ASGVehicle->GetNumStructuralSceneActorsHit() > 0)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_VEHICLE_COLLISION."));
		ResetVehicleAndSendAnalyzeScenarioRequest(ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest::REASON_VEHICLE_COLLISION);
		return true;
	}

	// Did vehicle roll over?
	if (ROSInst && FMath::Abs(ASGVehicle->GetActorRotation().Euler().X) > MaxVehicleRoll)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_VEHICLE_FLIPPED. Vehicle roll %f degrees exceeds %f degree maximum."), ASGVehicle->GetActorRotation().Euler().X, MaxVehicleRoll);
		ResetVehicleAndSendAnalyzeScenarioRequest(ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest::REASON_VEHICLE_FLIPPED);
		return true;
	}

	// Did vehicle flip over?
	if (ROSInst && FMath::Abs(ASGVehicle->GetActorRotation().Euler().Y) > MaxVehiclePitch)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_VEHICLE_FLIPPED. Vehicle pitch %f degrees exceeds %f degree maximum."), ASGVehicle->GetActorRotation().Euler().Y, MaxVehiclePitch);
		ResetVehicleAndSendAnalyzeScenarioRequest(ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest::REASON_VEHICLE_FLIPPED);
		return true;
	}

	// Did simulation timer expire?
	if (ROSInst && SimTimeoutPeriod > 0. && ASGVehicle->GetTimeSinceFirstControlInput() >= SimTimeoutPeriod)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_SIM_TIMEOUT. Simulation timeout is %f seconds."), SimTimeoutPeriod);
		ResetVehicleAndSendAnalyzeScenarioRequest(ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest::REASON_SIM_TIMEOUT);
		return true;
	}

	// Is vehicle idling too long?
	if (ROSInst && VehicleIdlingTimeoutPeriod > 0. && ASGVehicle->GetIdleTime() >= VehicleIdlingTimeoutPeriod)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_VEHICLE_IDLING_TIMEOUT. Vehicle idling timeout is %f seconds."), VehicleIdlingTimeoutPeriod);
		ResetVehicleAndSendAnalyzeScenarioRequest(ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest::REASON_VEHICLE_IDLING_TIMEOUT);
		return true;
	}

	// Is vehicle stuck for too long?
	if (ROSInst && VehicleStuckTimeoutPeriod > 0. && ASGVehicle->GetStuckTime() >= VehicleStuckTimeoutPeriod)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_VEHICLE_STUCK_TIMEOUT. Vehicle stuck timeout is %f seconds."), VehicleIdlingTimeoutPeriod);
		ResetVehicleAndSendAnalyzeScenarioRequest(ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest::REASON_VEHICLE_STUCK_TIMEOUT);
		return true;
	}

	return false;
}

bool AAutoSceneGenWorker::CheckGoalLocation() 
{
	if (!ASGVehicle || !ASGVehicle->IsEnabled()) return false;

	FVector DistanceToGoal = ASGVehicle->GetActorLocation() - VehicleGoalLocation;
	DistanceToGoal.Z = 0;
	if (DistanceToGoal.Size() <= GoalRadius)
	{
		UE_LOG(LogASG, Display, TEXT("Scenario has terminated: REASON_SUCCESS."));
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest> Req(new ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest());
		ASGVehicle->ResetVehicle(VehicleStartLocation, VehicleStartRotation, Req->vehicle_trajectory);

		if (ROSInst && !bWaitingForScenarioRequest /*&& bASGClientOnline*/)
		{
			bWaitingForScenarioRequest = true;
			WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;

			Req->worker_id = WorkerID;
			Req->scenario_number = ScenarioNumber;
			Req->termination_reason = Req->REASON_SUCCESS;

			FROSTime start_time = Req->vehicle_trajectory[0].header.time;
			FROSTime end_time = Req->vehicle_trajectory[Req->vehicle_trajectory.Num()-1].header.time;
			Req->vehicle_sim_time = FROSTime::GetTimeDelta(start_time, end_time);

			UE_LOG(LogASG, Display, TEXT("Computed vehicle sim time: %f"), Req->vehicle_sim_time);
			UE_LOG(LogASG, Display, TEXT("Submitting AnalyzeScenario request %i. ASG worker is ONLINE_AND_READY."), ScenarioNumber);
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

void AAutoSceneGenWorker::OnROSConnectionStatus(bool bIsConnected)
{
	if (!bROSBridgeHealthy && bIsConnected)
	{
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
		UE_LOG(LogASG, Display, TEXT("ROSBridge connection is healthy. ASG worker is ONLINE_AND_READY."));
	}
	else if (bROSBridgeHealthy && !bIsConnected)
	{
		bROSBridgeConnectionInterrupted = true;
		bASGClientOnline = false;
		bWaitingForScenarioRequest = true;
		bForceVehicleReset = true;
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::OFFLINE;
		UE_LOG(LogASG, Warning, TEXT("ROSBridge connection was interrupted. ASG worker is OFFLINE."));
	}
	bROSBridgeHealthy = bIsConnected;
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
		UE_LOG(LogASG, Display, TEXT("ASG client is online"));
	}

	// We assume the ASG client is configured to send an offline signal whenever it gets shutdown
	if (bASGClientOnline && CastMsg->status != ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_RUNNING)
	{
		bForceVehicleReset = true; // This will reset the run for us
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
		UE_LOG(LogASG, Warning, TEXT("ASG client is offline"));
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

void AAutoSceneGenWorker::WorkerIssueNotificationResponseCB(TSharedPtr<FROSBaseServiceResponse> Response) 
{
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationResponse>(Response);
	if (!CastResponse)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast msg to auto_scene_gen_msgs/WorkerIssueNotificationResponse"));
		return;
	}
	UE_LOG(LogASG, Display, TEXT("WorkerIssueNotification request received: %s"), (CastResponse->received ? *FString("True") : *FString("False")));
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

	SimTimeoutPeriod = CastRequest->sim_timeout_period;
	VehicleIdlingTimeoutPeriod = CastRequest->vehicle_idling_timeout_period;
	VehicleStuckTimeoutPeriod = CastRequest->vehicle_stuck_timeout_period;
	bAllowCollisions = CastRequest->allow_collisions;

	VehicleStartLocation.X = CastRequest->vehicle_start_location.x;
	VehicleStartLocation.Y = CastRequest->vehicle_start_location.y;
	VehicleStartRotation = FRotator(0, CastRequest->vehicle_start_yaw, 0);
	VehicleGoalLocation.X = CastRequest->vehicle_goal_location.x;
	VehicleGoalLocation.Y = CastRequest->vehicle_goal_location.y;
	GoalRadius = CastRequest->goal_radius;

	SceneDescription = CastRequest->scene_description;

	bWaitingForScenarioRequest = false; // Can only set this to false here
	bProcessedScenarioRequest = false;
	WorkerStatus = ROSMessages::auto_scene_gen_msgs::StatusCode::ONLINE_AND_READY;
	UE_LOG(LogASG, Display, TEXT("Saved scenario description %i"), ScenarioNumber);

	// Fill in response
	CastResponse->received = true;
}
