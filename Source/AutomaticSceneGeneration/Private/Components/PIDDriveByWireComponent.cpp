// Fill out your copyright notice in the Description page of Project Settings.


#include "Components/PIDDriveByWireComponent.h"
#include "Actors/AutoSceneGenWorker.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "WheeledVehicle.h"
#include "WheeledVehicleMovementComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "auto_scene_gen_logging.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"
#include "vehicle_msgs/PhysXControl.h"

// Sets default values for this component's properties
UPIDDriveByWireComponent::UPIDDriveByWireComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics; // Use PrePhysics because this actor handles control inputs
}

// Called when the game starts
void UPIDDriveByWireComponent::BeginPlay()
{
	Super::BeginPlay();

	bEnabled = false;
	bBypassController = false; // Updated based on what control messages arrive
	bProcessedBypassControlInput = true;

	MaxSteeringAngle = 0.f;
	DesiredVelocity = 0.f; // [m/s]
	DesiredSteeringAngle = 0.f; // [deg]

	Vehicle = Cast<AAutoSceneGenVehicle>(GetOwner());
	if (!Vehicle)
	{
		UE_LOG(LogASG, Error, TEXT("Drive by wire compent owner must be a AAutoSceneGenVehicle."));
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
		UE_LOG(LogASG, Warning, TEXT("Max steering angle is 0 degrees."));
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
		PhysxControllerSub->Init(ROSInst->ROSIntegrationCore, PhysxTopic, TEXT("vehicle_msgs/PhysXControl"));
		PhysxControllerSub->Subscribe(std::bind(&UPIDDriveByWireComponent::PhysxControllerCB, this, std::placeholders::_1));
		UE_LOG(LogASG, Display, TEXT("Initialized PID drive-by-wire ROS subscriber: %s"), *PhysxTopic);
	}
}

void UPIDDriveByWireComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);
	if (ROSInst)
	{
		BypassSub->Unsubscribe();
		PhysxControllerSub->Unsubscribe();
	}
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
}

bool UPIDDriveByWireComponent::IsManualDrive() const
{
	return bManualDrive;
}

float UPIDDriveByWireComponent::GetMaxManualDriveSpeed() const
{
	return MaxManualDriveSpeed;
}

float UPIDDriveByWireComponent::GetMaxSteeringAngle() const
{
	return MaxSteeringAngle;
}

void UPIDDriveByWireComponent::EnableDriveByWire(bool bEnable) 
{
	bEnabled = bEnable;
	SetDesiredForwardVelocity(0.f);
	SetDesiredSteeringAngle(0.f);
	VehicleMovementComponent->SetThrottleInput(0.f);
	VehicleMovementComponent->SetSteeringInput(0.f);
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
		UE_LOG(LogASG, Warning, TEXT("Failed to cast msg to geometry_msgs/Pose."));
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
	auto CastMsg = StaticCastSharedPtr<ROSMessages::vehicle_msgs::PhysXControl>(Msg);
	if (!CastMsg)
	{
		UE_LOG(LogASG, Warning, TEXT("Failed to cast msg to vehicle_msgs/PhysXControl."));
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
}

void UPIDDriveByWireComponent::SetThrottleInput(float DeltaTime)
{
	if (!bEnabled || bBypassController|| !VehicleMovementComponent)
	{
		return;
	}
	
	float Error = DesiredVelocity - VehicleMovementComponent->GetForwardSpeed()/100.f; // Error in [m/s]
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