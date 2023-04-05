// Fill out your copyright notice in the Description page of Project Settings.


#include "Components/PIDDriveByWireComponent.h"
#include "Actors/AutoSceneGenWorker.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "WheeledVehicle.h"
#include "WheeledVehicleMovementComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "AutoSceneGenLogging.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"
#include "auto_scene_gen_msgs/msg/PhysXControl.h"

// Sets default values for this component's properties
UPIDDriveByWireComponent::UPIDDriveByWireComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics; // Use PrePhysics because this actor handles control inputs
}

// Called when the game starts
void UPIDDriveByWireComponent::BeginPlay()
{
	Super::BeginPlay();

	bEnabled = false;
	bReceivedFirstControlInput = false;
	bBypassController = false; // Updated based on what control messages arrive
	bProcessedBypassControlInput = true;
	NonROSTimeSinceFirstControl = 0.f;
	NumRemoteControlMessagesReceived = 0;

	MaxSteeringAngle = 0.f;
	DesiredVelocity = 0.f; // [m/s]
	DesiredSteeringAngle = 0.f; // [deg]

	Vehicle = Cast<AAutoSceneGenVehicle>(GetOwner());
	if (!Vehicle)
	{
		UE_LOG(LogASG, Error, TEXT("Drive-by-wire component owner must be an AutoSceneGenVehicle."));
		return;
	}

	VehicleMovementComponent = Vehicle->GetVehicleMovementComponent();
	if (!VehicleMovementComponent)
	{
		UE_LOG(LogASG, Error, TEXT("Could not get vehicle movement component."));
		return;
	}

	for (UVehicleWheel* Wheel: VehicleMovementComponent->Wheels)
	{
		MaxSteeringAngle = FMath::Max(MaxSteeringAngle, Wheel->SteerAngle);
	}
	
	if (MaxSteeringAngle == 0.f)
	{
		UE_LOG(LogASG, Warning, TEXT("Max steering angle is 0 degrees. Check your wheel setups in the movement component."));
	}

	ROSInst = Cast<UROSIntegrationGameInstance>(GetOwner()->GetGameInstance());
	if (ROSInst)
	{
		// Create topic prefix
		FString TopicPrefix = FString::Printf(TEXT("/%s/control/"), *Vehicle->GetVehicleName());
		
		// Find ASG Worker ID, if applicable
		TArray<AActor*> TempArray;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAutoSceneGenWorker::StaticClass(), TempArray);
		if (TempArray.Num() > 0)
		{
			AAutoSceneGenWorker* Worker = Cast<AAutoSceneGenWorker>(TempArray[0]);
			if (Worker)
			{
				TopicPrefix = FString::Printf(TEXT("/asg_worker%i"), Worker->GetWorkerID()) + TopicPrefix;
			}
		}
		
		BypassSub =  NewObject<UTopic>(UTopic::StaticClass());
		PhysxControllerSub =  NewObject<UTopic>(UTopic::StaticClass());

		FString BypassTopic = TopicPrefix + FString("bypass");
		BypassSub->Init(ROSInst->ROSIntegrationCore, BypassTopic, TEXT("geometry_msgs/Pose"));
		BypassSub->Subscribe(std::bind(&UPIDDriveByWireComponent::BypassControllerCB, this, std::placeholders::_1));
		UE_LOG(LogASG, Display, TEXT("Initialized PID drive-by-wire ROS subscriber: %s"), *BypassTopic);

		FString PhysxTopic = TopicPrefix + FString("physx");
		PhysxControllerSub->Init(ROSInst->ROSIntegrationCore, PhysxTopic, TEXT("auto_scene_gen_msgs/PhysXControl"));
		PhysxControllerSub->Subscribe(std::bind(&UPIDDriveByWireComponent::PhysxControllerCB, this, std::placeholders::_1));
		UE_LOG(LogASG, Display, TEXT("Initialized PID drive-by-wire ROS subscriber: %s"), *PhysxTopic);
	}
}

void UPIDDriveByWireComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);
}

// Called every frame
void UPIDDriveByWireComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// If vehicle is in manual drive, then the vehicle will set the desired forward speed, steering, and handbrake.
	if (!bEnabled || !VehicleMovementComponent)
	{
		return;
	}

	if (bBypassController)
	{
		ProcessBypassControlInput();
	}
	else
	{
		SetThrottleInput(DeltaTime);
		SetSteeringInput(DeltaTime);
	}

	if (bEnabled && bReceivedFirstControlInput)
		NonROSTimeSinceFirstControl += DeltaTime;
}

bool UPIDDriveByWireComponent::IsManualDrive() const
{
	return bManualDrive;
}

bool UPIDDriveByWireComponent::ReceivedFirstControlInput() const
{
	return bReceivedFirstControlInput;
}

float UPIDDriveByWireComponent::GetTimeSinceFirstControlInput() const
{
	if (!bEnabled || !bReceivedFirstControlInput)
		return 0.f;
	
	if (ROSInst)
		return FROSTime::GetTimeDelta(ROSTimeAtFirstControl, FROSTime::Now());
	else
		return NonROSTimeSinceFirstControl;
}

int32 UPIDDriveByWireComponent::GetNumRemoteControlMessagesReceived() const
{
	return NumRemoteControlMessagesReceived;
}

float UPIDDriveByWireComponent::GetForwardSpeed() const
{
	return VehicleMovementComponent->GetForwardSpeed();
}

float UPIDDriveByWireComponent::GetMaxManualDriveSpeed() const
{
	return MaxManualDriveSpeed;
}

float UPIDDriveByWireComponent::GetMaxSteeringAngle() const
{
	return MaxSteeringAngle;
}

float UPIDDriveByWireComponent::GetDesiredVelocity() const
{
	return DesiredVelocity;
}

float UPIDDriveByWireComponent::GetDesiredSteeringAngle() const
{
	return DesiredSteeringAngle;
}

void UPIDDriveByWireComponent::EnableDriveByWire(bool bEnable) 
{
	bEnabled = bEnable;
	if (bEnabled)
		NumRemoteControlMessagesReceived = 0;
	
	if (!bEnabled)
		bReceivedFirstControlInput = false;
	
	if (bEnabled && bManualDrive && !bReceivedFirstControlInput)
	{
		bReceivedFirstControlInput = true;
		NonROSTimeSinceFirstControl = 0.f;
		UE_LOG(LogASG, Display, TEXT("PID drive-by-wire controller has received first manual control input."));
	}

	SetDesiredForwardVelocity(0.f);
	SetDesiredSteeringAngle(0.f);
	VehicleMovementComponent->SetThrottleInput(0.f);
	VehicleMovementComponent->SetSteeringInput(0.f);
	VehicleMovementComponent->SetHandbrakeInput(true);
}

void UPIDDriveByWireComponent::SetDesiredForwardVelocity(float NewVelocity) 
{
	DesiredVelocity = NewVelocity;
}

void UPIDDriveByWireComponent::SetDesiredSteeringAngle(float NewSteeringAngle) 
{
	DesiredSteeringAngle = FMath::Clamp<float>(NewSteeringAngle, -MaxSteeringAngle, MaxSteeringAngle);
}

void UPIDDriveByWireComponent::SetHandbrakeInput(bool bEngaged)
{
	VehicleMovementComponent->SetHandbrakeInput(bEngaged);
}

void UPIDDriveByWireComponent::BypassControllerCB(TSharedPtr<FROSBaseMsg> Msg) 
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(Msg);
	if (!CastMsg)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast msg to geometry_msgs/Pose."));
		bBypassController = false;
		return;
	}
	if (!bEnabled)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle is not enabled. Ignoring bypass controller request."));
		bBypassController = false;
		return;
	}
	if (bManualDrive)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle is still in manual drive. Cannot use bypass controller."));
		bBypassController = false;
		return;
	}

	BypassLocation.X = CastMsg->position.x;
	BypassLocation.Y = CastMsg->position.y;
	BypassLocation.Z = Vehicle->GetNominalVehicleZLocation();
	BypassQuat = FQuat(CastMsg->orientation.x, CastMsg->orientation.y, CastMsg->orientation.z, CastMsg->orientation.w);

	bBypassController = true;
	bProcessedBypassControlInput = false;
}

void UPIDDriveByWireComponent::PhysxControllerCB(TSharedPtr<FROSBaseMsg> Msg) 
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::PhysXControl>(Msg);
	if (!CastMsg)
	{
		UE_LOG(LogASG, Error, TEXT("Failed to cast msg to auto_scene_gen_msgs/PhysXControl."));
		return;
	}
	if (!bEnabled)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle is not enabled. Ignoring PhysxControl message."));
		return;
	}
	if (bManualDrive)
	{
		UE_LOG(LogASG, Warning, TEXT("Vehicle is still in manual drive. Cannot use PID drive-by-wire controller."));
		return;
	}

	SetDesiredForwardVelocity(CastMsg->longitudinal_velocity);
	SetDesiredSteeringAngle(CastMsg->steering_angle);
	SetHandbrakeInput(CastMsg->handbrake);
	bBypassController = false;
	NumRemoteControlMessagesReceived += 1;

	if (bEnabled && !bReceivedFirstControlInput)
	{
		bReceivedFirstControlInput = true;
		NonROSTimeSinceFirstControl = 0.f;
		if (ROSInst)
			ROSTimeAtFirstControl = FROSTime::Now();
		UE_LOG(LogASG, Display, TEXT("PID drive-by-wire controller has received first remote control input."));
	}
}

void UPIDDriveByWireComponent::SetThrottleInput(float DeltaTime)
{
	if (!bEnabled || bBypassController|| !VehicleMovementComponent)
	{
		return;
	}
	
	float Error = DesiredVelocity - VehicleMovementComponent->GetForwardSpeed(); // Error in [cm/s]
	float ThrottleInput = KpThrottle * Error + KdThrottle * Error / DeltaTime;
	VehicleMovementComponent->SetThrottleInput(ThrottleInput);
}

void UPIDDriveByWireComponent::SetSteeringInput(float DeltaTime)
{
	if (!bEnabled || bBypassController || !VehicleMovementComponent)
	{
		return;
	}
	
	float SteeringInput = FMath::Clamp<float>(DesiredSteeringAngle / MaxSteeringAngle, -1.f, 1.f);
	VehicleMovementComponent->SetSteeringInput(SteeringInput);
}

void UPIDDriveByWireComponent::ProcessBypassControlInput() 
{
	if (bBypassController && !bProcessedBypassControlInput)
	{
		Vehicle->SetActorLocationAndRotation(BypassLocation, BypassQuat, false, nullptr, ETeleportType::TeleportPhysics);
		Vehicle->GetMesh()->SetAllPhysicsAngularVelocityInDegrees(FVector(0.f));
		bProcessedBypassControlInput = true;
	}
}