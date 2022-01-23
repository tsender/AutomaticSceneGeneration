// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/AutoSceneGenWorker.h"
#include "Actors/StructuralSceneActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/TriggerVolume.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/RI/Service.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/std_msgs/Bool.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"

#include "auto_scene_gen_msgs/WorkerStatus.h"
#include "auto_scene_gen_msgs/RunScenarioRequest.h"
#include "auto_scene_gen_msgs/RunScenarioResponse.h"
#include "auto_scene_gen_msgs/AnalyzeScenarioRequest.h"
#include "auto_scene_gen_msgs/AnalyzeScenarioResponse.h"

DEFINE_LOG_CATEGORY(LogASG);

// Sets default values
AAutoSceneGenWorker::AAutoSceneGenWorker()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AAutoSceneGenWorker::BeginPlay()
{
	Super::BeginPlay();
	// UGameplayStatics::SetGlobalTimeDilation(GetWorld(), GlobalTimeDilation);

	WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;
	ScenarioNumber = 0;
	bASGOnline = false;
	bASGStatusClockInit = false;
	bForceVehicleReset = false;
	bWaitingForScenarioRequest = false;
	bProcessedScenarioRequest = true;
	bReadyToTick = false;
	bDoneTesting = false;

	// Find player start actor
	TArray<AActor*> TempArray;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), APlayerStart::StaticClass(), TempArray);
	if (!TempArray.Num())
	{
		UE_LOG(LogASG, Error, TEXT("Could not find any Player Start Actors."));
		return;
	}
    PlayerStart = Cast<APlayerStart>(TempArray[0]);

	// Find ground plane actor
	TempArray.Empty();
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), TempArray);
	if (!TempArray.Num())
	{
		UE_LOG(LogASG, Error, TEXT("Could not find any Static Mesh Actors."));
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

	// Find evaluation vehicle
	ASGVehicle = Cast<AAutoSceneGenVehicle>(GetWorld()->GetFirstPlayerController()->GetPawn());
	if (!ASGVehicle)
	{
		UE_LOG(LogASG, Error, TEXT("Could not get AAutoSceneGenVehicle from first player controller."));
		return;
	}
	ASGVehicle->SetDefaultResetInfo(PlayerStart->GetActorLocation(), PlayerStart->GetActorRotation());

	// Get SSA subclasses
	NumSSASubclasses = StructuralSceneActorSubclasses.Num();
	if (!NumSSASubclasses)
	{
		UE_LOG(LogASG, Error, TEXT("Must provide at least 1 structural scene actor type to ASG game client."));
		return;
	}

	SSADataArraySize = EStructuralSceneAttribute::Size * NumSSASubclasses * NumSSAInstances;
	SSADataArray.Reset(SSADataArraySize);

	InitStructuralSceneActorArray();

	ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	if (ROSInst)
	{
		ASGStatusSub = NewObject<UTopic>(UTopic::StaticClass());
		FString ASGStatusTopic = FString("/asg/status");
		ASGStatusSub->Init(ROSInst->ROSIntegrationCore, ASGStatusTopic, TEXT("std_msgs/Bool"));
		ASGStatusSub->Subscribe(std::bind(&AAutoSceneGenWorker::ASGStatusCB, this, std::placeholders::_1));
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS subscriber: %s"), *ASGStatusTopic);

		WorkerStatusPub = NewObject<UTopic>(UTopic::StaticClass());
		FString WorkerStatusTopic = FString::Printf(TEXT("/asg_worker%i/status"), WorkerID);
		WorkerStatusPub->Init(ROSInst->ROSIntegrationCore, WorkerStatusTopic, TEXT("auto_scene_gen_msgs/WorkerStatus"));
		WorkerStatusPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS publisher: %s"), *WorkerStatusTopic);

		VehicleDestinationPub = NewObject<UTopic>(UTopic::StaticClass());
		FString VehicleDestinationTopic = FString::Printf(TEXT("/asg_worker%i/nav/destination"), WorkerID);
		VehicleDestinationPub->Init(ROSInst->ROSIntegrationCore, VehicleDestinationTopic, TEXT("geometry_msgs/Pose"));
		VehicleDestinationPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized ASG worker ROS publisher: %s"), *VehicleDestinationTopic);
		
		AnalyzeScenarioClient = NewObject<UService>(UService::StaticClass());
		FString AnalyzeScenarioClientName = FString("/asg/services/analyze_scenario");
		AnalyzeScenarioClient->Init(ROSInst->ROSIntegrationCore, AnalyzeScenarioClientName, TEXT("auto_scene_gen_msgs/AnalyzeScenario"));
		UE_LOG(LogASG, Display, TEXT("Registered ASG worker analyze scenario ROS client to: %s"), *AnalyzeScenarioClientName);

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

	// Calling this here forces the engine to render everything before the tick function
	RandomizeStructuralSceneActors();
}

void AAutoSceneGenWorker::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);
	if (ROSInst)
	{
		// Publish status as offline 10 times to ensure at least 1 or 2 messages get recceived before the game ends
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::WorkerStatus> StatusMsg(new ROSMessages::auto_scene_gen_msgs::WorkerStatus( ROSMessages::auto_scene_gen_msgs::WorkerStatus::OFFLINE));
		for (int32 I = 0; I < 10; I++)
		{
			WorkerStatusPub->Publish(StatusMsg);
			FPlatformProcess::Sleep(0.1f);
		}
		WorkerStatusPub->Unadvertise();

		ASGStatusSub->Unsubscribe();
		VehicleDestinationPub->Unadvertise();
		RunScenarioService->Unadvertise();
	}
}

// Called every frame
void AAutoSceneGenWorker::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bForceVehicleReset)
	{
		UE_LOG(LogASG, Warning, TEXT("Forcing vehicle rest"));
		ASGVehicle->ResetVehicle(PlayerStart->GetActorLocation(), PlayerStart->GetActorRotation());
		bForceVehicleReset = false;
	}
	
	if (!bProcessedScenarioRequest)
	{
		ProcessScenarioRequest();
	}
	
	// This forces the worker to wait one tick for the frame to render before publishing its status
	if (!bReadyToTick)
	{
		bReadyToTick = true;
		ASGVehicle->SetWorldIsReadyFlag(true);
		return;
	}

	if (bProcessedScenarioRequest && !bWaitingForScenarioRequest)
	{
		CheckIfVehicleCrashed();
		CheckIfVehicleTurnedOver();
		CheckTriggerVolume();
	}

	if (ROSInst)
	{
		// Publish ASG worker status
		TSharedPtr<ROSMessages::auto_scene_gen_msgs::WorkerStatus> StatusMsg(new ROSMessages::auto_scene_gen_msgs::WorkerStatus(WorkerStatus));
		WorkerStatusPub->Publish(StatusMsg);
		
		// Publish vehicle destination info
		TSharedPtr<ROSMessages::geometry_msgs::Pose> DestMsg(new ROSMessages::geometry_msgs::Pose());
		ROSMessages::geometry_msgs::Point Location(TriggerVolume->GetActorLocation()/100.f); // Put into [m]
		Location.y *= -1;
		DestMsg->position = Location;
		ROSMessages::geometry_msgs::Quaternion Quaternion;
		Quaternion = TriggerVolume->GetActorQuat();
		Quaternion.x *= -1;
		Quaternion.z *= -1;
		DestMsg->orientation = Quaternion;
		VehicleDestinationPub->Publish(DestMsg);

		// Occasionaly, for some unknown reason, calling reset can throw the vehicle out of the bounds of the game and we need a way to recover from this.
		// Make sure to uncheck EnableWorldBoundsCheck in world settings, since we do not want UE4 deleting the vehicle from the game
		if (ASGVehicle->GetActorLocation().Z <= GroundPlaneZHeight -100.)
		{
			UE_LOG(LogASG, Warning, TEXT("Vehicle was thrown out of bounds or off the landscape. Resetting so we can try again."));
			bProcessedScenarioRequest = false; // This will reset the run for us
			WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;
			return;
		}
	}

	if (bDoneTesting)
	{
		UE_LOG(LogASG, Display, TEXT("All done testing. Ending the game."));
		UGameplayStatics::GetPlayerController(GetWorld(), 0)->ConsoleCommand("quit");
		return;
	}
}

uint8 AAutoSceneGenWorker::GetWorkerID() const
{
	return WorkerID;
}

TArray<float> AAutoSceneGenWorker::GetSSAAttributes(uint16 Subclass, uint16 Index) const
{
	TArray<float> Data;

	if (Subclass >= NumSSASubclasses)
	{
		UE_LOG(LogASG, Error, TEXT("Subclass index %i is out of bounds"), Subclass);
		return Data;
	}

	int32 StartIndex = EStructuralSceneAttribute::Size * NumSSAInstances * Subclass + EStructuralSceneAttribute::Size  * Index;
	for (int32 i = 0; i < EStructuralSceneAttribute::Size; i++)
	{
		Data.Emplace(SSADataArray[StartIndex + i]);
	}
	return Data;
}

void AAutoSceneGenWorker::InitStructuralSceneActorArray() 
{
	// Populate SSADataArray
	for (int32 i = 0; i < NumSSASubclasses * NumSSAInstances; i++)
	{
		SSADataArray.Emplace(1.f); // Visibility
		SSADataArray.Emplace(0.f); // X
		SSADataArray.Emplace(0.f); // Y
		SSADataArray.Emplace(0.f); // Yaw
		SSADataArray.Emplace(1.f); // Scale, CANNOT be close to 0
	}

	// Spawn structural scene actors and store pointers
	for (int32 i = 0; i < NumSSASubclasses; i++)
	{
		for (int32 j = 0; j < NumSSAInstances; j++)
		{
			FVector Location(0.f, 0.f, GroundPlaneZHeight);
			FRotator Rotation(0.f, 0.f, 0.f);
			FActorSpawnParameters SpawnParams;
			AStructuralSceneActor* Actor = GetWorld()->SpawnActor<AStructuralSceneActor>(StructuralSceneActorSubclasses[i], Location, Rotation);

			Actor->SetStructuralAttributes(GetSSAAttributes(i,j));
			Actor->SetOwner(this);
			StructuralSceneActorArray.Emplace(Actor);
		}
	}
	bSSAInit = true;
}

// For debugging purposes 
void AAutoSceneGenWorker::RandomizeStructuralSceneActors()
{
	if (!bSSAInit)
	{
		return;
	}

	for (int32 i = 0; i < NumSSASubclasses; i++)
	{
		for (int32 j = 0; j < NumSSAInstances; j++)
		{
			int32 Index = NumSSAInstances*i + j;
			TArray<float> NewData = {1.f, //FMath::RandRange(0.f, 1.f), 
									FMath::RandRange(0.f, 1.f) * LandscapeSize.X, 
									-FMath::RandRange(0.f, 1.f) * LandscapeSize.Y,
									FMath::RandRange(0.f, 1.f) * 360.f,
									1., //FMath::RandRange(0.5f, 1.f) // Scale cannot be too small, else rendering problems
									};
			FVector Loc = FVector(NewData[1], NewData[2], PlayerStart->GetActorLocation().Z);
			if ((Loc - PlayerStart->GetActorLocation()).Size() <= SafetyRadius)
			{
				NewData[0] = 0;
			}
			StructuralSceneActorArray[Index]->SetStructuralAttributes(NewData);
		}
	}
}

bool AAutoSceneGenWorker::CheckIfVehicleTurnedOver()
{
	if (!ASGVehicle->IsEnabled()) return false;

	if (FMath::Abs(ASGVehicle->GetActorRotation().Euler().X) > 70.f || FMath::Abs(ASGVehicle->GetActorRotation().Euler().Y) > 70.f)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle roll or pitch is too large. Resetting vehicle."));
		ASGVehicle->ResetVehicle(PlayerStart->GetActorLocation(), PlayerStart->GetActorRotation());
		ASGVehicle->SetWorldIsReadyFlag(true);
		return true;
	}
	return false;
}

bool AAutoSceneGenWorker::CheckIfVehicleCrashed()
{
	if (ROSInst && ASGVehicle->GetNumSSAHit() > 0)
	{
		ASGVehicle->ResetVehicle(PlayerStart->GetActorLocation(), PlayerStart->GetActorRotation());
		ROSMessages::nav_msgs::Path VehiclePath;
		ASGVehicle->GetVehiclePath(VehiclePath);
		
		bWaitingForScenarioRequest = true;
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;
		TSharedPtr<auto_scene_gen_msgs::FAnalyzeScenarioRequest> Req(new auto_scene_gen_msgs::FAnalyzeScenarioRequest());
		Req->worker_id = WorkerID;
		Req->scenario_number = ScenarioNumber;
		Req->crashed = true;
		Req->succeeded = false;
		Req->vehicle_path = VehiclePath;

		UE_LOG(LogASG, Warning, TEXT("Submitting analyze scenario request %i"), ScenarioNumber);
		AnalyzeScenarioClient->CallService(Req, std::bind(&AAutoSceneGenWorker::AnalyzeScenarioResponseCB, this, std::placeholders::_1));
		return true;
	}
	return false;
}

bool AAutoSceneGenWorker::CheckTriggerVolume() 
{
	TArray<AActor*> OverlappingActors;
    if (!TriggerVolume) return false;
    TriggerVolume->GetOverlappingActors(OverlappingActors);

	for (AActor* Actor: OverlappingActors)
	{
		AAutoSceneGenVehicle* Veh = Cast<AAutoSceneGenVehicle>(Actor);
		if (Veh)
		{
			Veh->ResetVehicle(PlayerStart->GetActorLocation(), PlayerStart->GetActorRotation());
			ROSMessages::nav_msgs::Path VehiclePath;
			Veh->GetVehiclePath(VehiclePath);

			if (ROSInst && bASGOnline && !bWaitingForScenarioRequest)
			{
				bWaitingForScenarioRequest = true;
				WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;

				TSharedPtr<auto_scene_gen_msgs::FAnalyzeScenarioRequest> Req(new auto_scene_gen_msgs::FAnalyzeScenarioRequest());
				Req->worker_id = WorkerID;
				Req->scenario_number = ScenarioNumber;
				Req->crashed = false;
				Req->succeeded = true;
				Req->vehicle_path = VehiclePath;

				UE_LOG(LogASG, Display, TEXT("Submitting analyze scenario request %i"), ScenarioNumber);
				AnalyzeScenarioClient->CallService(Req, std::bind(&AAutoSceneGenWorker::AnalyzeScenarioResponseCB, this, std::placeholders::_1));
				return true;
			}
			if (!ROSInst)
			{
				RandomizeStructuralSceneActors();
				UE_LOG(LogASG, Display, TEXT("ASG offline. Generating new random scene."));
			}
		}
	}
	return false;
}

void AAutoSceneGenWorker::ProcessScenarioRequest() 
{
	if (!ROSInst || !bASGOnline || bProcessedScenarioRequest) return;

	ASGVehicle->ResetVehicle(PlayerStart->GetActorLocation(), PlayerStart->GetActorRotation());
	
	for (int32 i = 0; i < NumSSASubclasses; i++)
	{
		for (int32 j = 0; j < NumSSAInstances; j++)
		{
			int32 Index = NumSSAInstances*i + j;
			StructuralSceneActorArray[Index]->SetStructuralAttributes(GetSSAAttributes(i,j));
		}
	}
	
	bReadyToTick = false;
	bProcessedScenarioRequest = true;
	WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_RUNNING;
	UE_LOG(LogASG, Display, TEXT("Processed scenario description %i"), ScenarioNumber);
}

void AAutoSceneGenWorker::ASGStatusCB(TSharedPtr<FROSBaseMsg> Msg) 
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::std_msgs::Bool>(Msg);
	if (!CastMsg)
	{
		UE_LOG(LogASG, Warning, TEXT("Failed to cast msg to std_msgs/Bool"));
		return;
	}

	if (!bASGOnline && CastMsg->_Data)
	{
		UE_LOG(LogASG, Warning, TEXT("ASG back online"));
	}

	if (bASGOnline && !CastMsg->_Data)
	{
		bForceVehicleReset = true; // This will reset the run for us
		// bProcessedScenarioRequest = false;
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;
		UE_LOG(LogASG, Warning, TEXT("ASG went offline"));
	}
	
	bASGOnline = CastMsg->_Data;

	// auto Now = std::chrono::steady_clock::now();
	// double MessagePeriod = 0.;

	// if (bASGStatusClockInit)
	// {
	// 	MessagePeriod = std::chrono::duration_cast<std::chrono::duration<double> >(Now - LastASGStatusClockTime).count();
	// }
	// LastASGStatusClockTime = Now;
	// bASGStatusClockInit = true;

	// // Make sure the ASG is functioning properly so we can communicate with it
	// if (MessagePeriod == 0.)
	// {
	// 	return;
	// }
	// if (MessagePeriod > 0. && MessagePeriod <= ASGStatusMessagePeriodThreshold)
	// {
	// 	if (!bASGOnline)
	// 	{
	// 		UE_LOG(LogASG, Display, TEXT("ASG back online"));
	// 	}
	// 	bASGOnline = true;
	// }
	// else if (MessagePeriod > ASGStatusMessagePeriodThreshold && bWaitingForScenarioRequest)
	// {
	// 	return; // Ignore the long wait from the scenario analysis
	// }
	// else
	// {
	// 	bASGOnline = false;
	// 	bASGStatusClockInit = false;
	// 	bProcessedScenarioRequest = false; // This will reset the run for us
	// 	WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;
	// 	UE_LOG(LogASG, Warning, TEXT("ASG is offline"));
	// }
}

void AAutoSceneGenWorker::AnalyzeScenarioResponseCB(TSharedPtr<FROSBaseServiceResponse> Response) 
{
	auto CastResponse = StaticCastSharedPtr<auto_scene_gen_msgs::FAnalyzeScenarioResponse>(Response);
	if (!CastResponse)
	{
		UE_LOG(LogASG, Warning, TEXT("Failed to cast msg to auto_scene_gen_msgs/AnalyzeScenario_Response"));
		return;
	}
	UE_LOG(LogASG, Display, TEXT("Analyze scenario request received: %s"), (CastResponse->received ? *FString("True") : *FString("False")));
}

void AAutoSceneGenWorker::RunScenarioServiceCB(TSharedPtr<FROSBaseServiceRequest> Request, TSharedPtr<FROSBaseServiceResponse> Response) 
{
	auto CastRequest = StaticCastSharedPtr<auto_scene_gen_msgs::FRunScenarioRequest>(Request);
	auto CastResponse = StaticCastSharedPtr<auto_scene_gen_msgs::FRunScenarioResponse>(Response);
	if (!CastRequest)
	{
		UE_LOG(LogASG, Warning, TEXT("Failed to cast Request to auto_scene_gen_msgs/RunScenario_Request"));
		return;
	}
	if (!CastResponse)
	{
		UE_LOG(LogASG, Warning, TEXT("Failed to cast Response to auto_scene_gen_msgs/RunScenario_Response"));
		return;
	}

	UE_LOG(LogASG, Display, TEXT("Received run scenario request %i"), CastRequest->scenario_number);

	if (!CastRequest->done_testing)
	{
		if (CastRequest->ssa_array.layout.dim.Num() == 0)
		{
			UE_LOG(LogASG, Warning, TEXT("No elements found in Float32MultiArray layout field"));
			return;
		}
		if (CastRequest->ssa_array.data.Num() == 0)
		{
			UE_LOG(LogASG, Warning, TEXT("No elements found in Float32MultiArray data field"));
			return;
		}
		if (CastRequest->ssa_array.layout.dim[0].stride != SSADataArraySize)
		{
			UE_LOG(LogASG, Warning, TEXT("Not enough elements in Float32MultiArray layout.dim[0].stride field. Expected %i but received %i."), SSADataArraySize, CastRequest->ssa_array.layout.dim[0].stride);
			return;
		}

		SSADataArray = CastRequest->ssa_array.data;
		ScenarioNumber = CastRequest->scenario_number;
		bWaitingForScenarioRequest = false;
		bProcessedScenarioRequest = false;
		WorkerStatus = ROSMessages::auto_scene_gen_msgs::WorkerStatus::ONLINE_AND_READY;
		UE_LOG(LogASG, Display, TEXT("Saved scenario description %i"), ScenarioNumber);
	}

	bDoneTesting = CastRequest->done_testing;

	// Fill in response
	CastResponse->received = true;
}
