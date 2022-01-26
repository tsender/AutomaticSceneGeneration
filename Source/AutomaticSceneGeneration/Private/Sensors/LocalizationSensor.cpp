// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/LocalizationSensor.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "Actors/AutoSceneGenWorker.h"
#include "Kismet/GameplayStatics.h"
#include "auto_scene_gen_logging.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"

// Sets default values for this component's properties
ULocalizationSensor::ULocalizationSensor()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics; // Sensors tick in PostPhysics
}

void ULocalizationSensor::BeginPlay()
{
	Super::BeginPlay();

	HeaderSequence = 1;

	ROSInst = Cast<UROSIntegrationGameInstance>(GetOwner()->GetGameInstance());
	if (ROSInst)
	{
		// Create topic prefix
		FString TopicPrefix = FString("/sensors/") + SensorName;
		HeaderFrameID = SensorName;

		AAutoSceneGenVehicle* Vehicle = Cast<AAutoSceneGenVehicle>(GetOwner());
		if (Vehicle)
		{
			TopicPrefix = FString::Printf(TEXT("/%s"), *Vehicle->GetVehicleName()) + TopicPrefix;
			HeaderFrameID = FString::Printf(TEXT("/%s/"), *Vehicle->GetVehicleName()) + SensorName;
		}

		// Find ASG Worker ID, if applicable
		TArray<AActor*> TempArray;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAutoSceneGenWorker::StaticClass(), TempArray);
		if (TempArray.Num() > 0)
		{
			AAutoSceneGenWorker* Worker = Cast<AAutoSceneGenWorker>(TempArray[0]);
			if (Worker)
			{
				TopicPrefix = FString::Printf(TEXT("/asg_worker%i"), Worker->GetWorkerID()) + TopicPrefix;
				HeaderFrameID = FString::Printf(TEXT("/asg_worker%i"), Worker->GetWorkerID()) + HeaderFrameID;
			}
		}

		SensorPub = NewObject<UTopic>(UTopic::StaticClass());

		FString LocTopic = TopicPrefix;
		SensorPub->Init(ROSInst->ROSIntegrationCore, LocTopic, TEXT("geometry_msgs/PoseStamped"));
		SensorPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized localization sensor ROS topic: %s"), *LocTopic);
	}

	if (bUseCustomFrameRate)
	{
		FTimerHandle TimerHandle;
		GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &ULocalizationSensor::TickSensor, 1./FrameRate, true);
	}
}

void ULocalizationSensor::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);
	if (ROSInst)
	{
		SensorPub->Unadvertise();
	}
}

void ULocalizationSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bUseCustomFrameRate)
	{
		return;
	}
	else
	{
		TickSensor();
	}
}

void ULocalizationSensor::TickSensor()
{
	if (ROSInst)
	{
		TSharedPtr<ROSMessages::geometry_msgs::PoseStamped> LocMsg(new ROSMessages::geometry_msgs::PoseStamped());
		LocMsg->header.seq = HeaderSequence;
		LocMsg->header.time = FROSTime::Now();
		LocMsg->header.frame_id = HeaderFrameID;

		ROSMessages::geometry_msgs::Point Location(GetComponentLocation()/100.f); // Put into [m]
		Location.y *= -1;
		LocMsg->pose.position = Location;

		ROSMessages::geometry_msgs::Quaternion Quaternion;
		Quaternion = GetComponentQuat();
		Quaternion.x *= -1;
		Quaternion.z *= -1;
		LocMsg->pose.orientation = Quaternion;
		SensorPub->Publish(LocMsg);
	}

	HeaderSequence++;
}