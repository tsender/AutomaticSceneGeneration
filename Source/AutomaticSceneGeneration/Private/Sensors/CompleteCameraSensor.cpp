// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/CompleteCameraSensor.h"
#include "Sensors/ColorCameraSensor.h"
#include "Sensors/DepthCameraSensor.h"
#include "Sensors/SegmentationCameraSensor.h"
#include "Components/AnnotationComponent.h"
#include "Actors/AutoSceneGenWorker.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "Kismet/GameplayStatics.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"

#include <cstring>

// Code to determine runtime
// auto start = std::chrono::steady_clock::now();
// // do something
// auto finish_gpu = std::chrono::steady_clock::now();
// double elapsed_seconds_gpu = std::chrono::duration_cast<std::chrono::duration<double> >(finish_gpu - start).count();

// Sets default values for this component's properties
UCompleteCameraSensor::UCompleteCameraSensor()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics; // Sensors tick in PostPhysics

	ColorCamera = CreateDefaultSubobject<UColorCameraSensor>(TEXT("Color Camera Sensor"));
	ColorCamera->SetupAttachment(this);

	DepthCamera = CreateDefaultSubobject<UDepthCameraSensor>(TEXT("Depth Camera Sensor"));
	DepthCamera->SetupAttachment(this);

	TravCamera = CreateDefaultSubobject<USegmentationCameraSensor>(TEXT("Traversability Camera Sensor"));
	TravCamera->SetupAttachment(this);

	SegCamera = CreateDefaultSubobject<USegmentationCameraSensor>(TEXT("Semantic Segmentation Camera Sensor"));
	SegCamera->SetupAttachment(this);
}

// Called when the game starts
void UCompleteCameraSensor::BeginPlay()
{
	Super::BeginPlay();

	// Checking for endianness
	unsigned int i = 1;
	char *c = (char*)&i;
	if (*c) // Little endian
	{
		bIsLittleEndian = true;
	}
	else // Big endian
	{
		bIsLittleEndian = false;
	}

	HeaderSequence = 1;
	bEnabled = true;

	EncodedColorData.resize(ImageWidth*ImageHeight*3);
	EncodedDepthData.resize(ImageWidth*ImageHeight*4);
	EncodedTravData.resize(ImageWidth*ImageHeight*3);
	EncodedSegData.resize(ImageWidth*ImageHeight*3);

	ColorCamera->InitTextureTarget(ImageWidth, ImageHeight);
	DepthCamera->InitTextureTarget(ImageWidth, ImageHeight);
	TravCamera->InitTextureTarget(ImageWidth, ImageHeight);
	SegCamera->InitTextureTarget(ImageWidth, ImageHeight);

	ColorCamera->SetSaveImages(bSaveImagesToDisk);
	TravCamera->SetSaveImages(bSaveImagesToDisk);
	SegCamera->SetSaveImages(bSaveImagesToDisk);

	ColorCamera->SetSavePrefix(FPaths::ProjectUserDir() + FString::Printf(TEXT("TrainingData/%s/color/color"), *SensorName));
	TravCamera->SetSavePrefix(FPaths::ProjectUserDir() + FString::Printf(TEXT("TrainingData/%s/trav/trav"), *SensorName));
	SegCamera->SetSavePrefix(FPaths::ProjectUserDir() + FString::Printf(TEXT("TrainingData/%s/seg/seg"), *SensorName));

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
		
		ColorCamPub = NewObject<UTopic>(UTopic::StaticClass());
		DepthCamPub = NewObject<UTopic>(UTopic::StaticClass());
		TravCamPub = NewObject<UTopic>(UTopic::StaticClass());
		SegCamPub = NewObject<UTopic>(UTopic::StaticClass());

		FString ColorTopic = TopicPrefix + FString("/color_image");
		ColorCamPub->Init(ROSInst->ROSIntegrationCore, ColorTopic, TEXT("sensor_msgs/Image"));
		ColorCamPub->Advertise();
		UE_LOG(LogTemp, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *ColorTopic);

		if (bEnableDepthCam)
		{
			FString DepthTopic = TopicPrefix + FString("/depth_image");
			DepthCamPub->Init(ROSInst->ROSIntegrationCore, DepthTopic, TEXT("sensor_msgs/Image"));
			DepthCamPub->Advertise();
			UE_LOG(LogTemp, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *DepthTopic);
		}
		
		if (bEnableTravCam)
		{
			FString TravTopic = TopicPrefix + FString("/trav_image");
			TravCamPub->Init(ROSInst->ROSIntegrationCore, TravTopic, TEXT("sensor_msgs/Image"));
			TravCamPub->Advertise();
			UE_LOG(LogTemp, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *TravTopic);
		}

		if (bEnableSegCam)
		{
			FString SegTopic = TopicPrefix + FString("/seg_image");
			SegCamPub->Init(ROSInst->ROSIntegrationCore, SegTopic, TEXT("sensor_msgs/Image"));
			SegCamPub->Advertise();
			UE_LOG(LogTemp, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *SegTopic);
		}
	}

	if (bUseCustomFrameRate)
	{
		FTimerHandle TimerHandle;
		GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &UCompleteCameraSensor::TickSensor, 1./FrameRate, true);
	}
}

void UCompleteCameraSensor::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);
	if (ROSInst)
	{
		ColorCamPub->Unadvertise();
		DepthCamPub->Unadvertise();
		TravCamPub->Unadvertise();
		SegCamPub->Unadvertise();
	}
}

// Called every frame
void UCompleteCameraSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bUseCustomFrameRate || !bEnabled)
	{
		return;
	}
	else
	{
		TickSensor();
	}
}

// Primary tick function for the camera sensor. Encode all message data in little endian format
void UCompleteCameraSensor::TickSensor()
{
	if (!bEnabled) // !ROSInst && !bSaveImagesToDisk || !bEnabled
	{
		return;
	}

	FROSTime ROSTime;
	if (ROSInst)
	{
		ROSTime = FROSTime::Now();
	}

	auto StartTickTime = std::chrono::steady_clock::now();

	// The depth camera returns the Z-component of the distance to each pixel in the frame
	if (bEnableDepthCam)
	{
		TArray<FFloat16Color> DepthData;
		DepthCamera->CaptureFloat16Color(DepthData);

		if (ROSInst)
		{
			// Encode depth data
			for (int32 i = 0; i < DepthData.Num(); ++i)
			{
				float d = DepthData[i].R.GetFloat() / 100.f; // Convert depth to float32 and then convert to [m]

				// Copy data to EncodedDepthData array with little endian order
				// If we only care about speed of data transfer, then we should instead just use the native byte ordering
				if (bIsLittleEndian)
				{
					// memcpy order: dest, src, size
					std::memcpy(&EncodedDepthData[i*4], &d, sizeof(float)); // Encode float32 as 4 bytes
				}
				else
				{
					uint8 Bytes[4];
					std::memcpy(Bytes, &d, sizeof(float)); // Encode float32 as 4 bytes
					uint8 LittleBytes[4] = {Bytes[3], Bytes[2], Bytes[1], Bytes[0]}; // Reverse byte order
					std::memcpy(&EncodedDepthData[i*4], LittleBytes, sizeof(float)); // Then copy to array
				}
			}

			TSharedPtr<ROSMessages::sensor_msgs::Image> DepthCamMsg(new ROSMessages::sensor_msgs::Image());
			DepthCamMsg->header.seq = HeaderSequence;
			DepthCamMsg->header.time = ROSTime;
			DepthCamMsg->header.frame_id = HeaderFrameID;
			DepthCamMsg->height = ImageHeight;
			DepthCamMsg->width = ImageWidth;
			DepthCamMsg->encoding = TEXT("32FC1"); // Each pixel encodes the float32 depth as 4 uint8 bytes in little endian format
			DepthCamMsg->is_bigendian = false;
			DepthCamMsg->step = ImageWidth * 4;
			DepthCamMsg->data = &EncodedDepthData[0];
			DepthCamPub->Publish(DepthCamMsg);
		}
	}
	
	if (bEnableTravCam)
	{
		TArray<FColor> TravData;
		TravCamera->CaptureColor(TravData, true, EAnnotationColor::Traversable);

		if (ROSInst)
		{
			// Encode trav data
			for (int32 i = 0; i < TravData.Num(); i++)
			{
				EncodedTravData[i*3] = TravData[i].R;
				EncodedTravData[i*3+1] = TravData[i].G;
				EncodedTravData[i*3+2] = TravData[i].B;
			}

			TSharedPtr<ROSMessages::sensor_msgs::Image> TravCamMsg(new ROSMessages::sensor_msgs::Image());
			TravCamMsg->header.seq = HeaderSequence;
			TravCamMsg->header.time = ROSTime;
			TravCamMsg->header.frame_id = HeaderFrameID;
			TravCamMsg->height = ImageHeight;
			TravCamMsg->width = ImageWidth;
			TravCamMsg->encoding = TEXT("rgb8");
			TravCamMsg->is_bigendian = false;
			TravCamMsg->step = ImageWidth * 3;
			TravCamMsg->data = &EncodedTravData[0];
			TravCamPub->Publish(TravCamMsg);
		}
	}

	if (bEnableSegCam)
	{
		TArray<FColor> SegData;
		SegCamera->CaptureColor(SegData, true, EAnnotationColor::SemanticSegmentation);

		if (ROSInst)
		{
			// Encode trav data
			for (int32 i = 0; i < SegData.Num(); i++)
			{
				EncodedSegData[i*3] = SegData[i].R;
				EncodedSegData[i*3+1] = SegData[i].G;
				EncodedSegData[i*3+2] = SegData[i].B;
			}

			TSharedPtr<ROSMessages::sensor_msgs::Image> SegCamMsg(new ROSMessages::sensor_msgs::Image());
			SegCamMsg->header.seq = HeaderSequence;
			SegCamMsg->header.time = ROSTime;
			SegCamMsg->header.frame_id = HeaderFrameID;
			SegCamMsg->height = ImageHeight;
			SegCamMsg->width = ImageWidth;
			SegCamMsg->encoding = TEXT("rgb8");
			SegCamMsg->is_bigendian = false;
			SegCamMsg->step = ImageWidth * 3;
			SegCamMsg->data = &EncodedSegData[0];
			SegCamPub->Publish(SegCamMsg);
		}
	}
	
	// Publish color image last b/c it sets the annotation material to default materials
	TArray<FColor> ColorData;
	ColorCamera->CaptureColor(ColorData, false);

	if (ROSInst)
	{
		// Encode color data
		for (int32 i = 0; i < ColorData.Num(); i++)
		{
			EncodedColorData[i*3] = ColorData[i].R;
			EncodedColorData[i*3+1] = ColorData[i].G;
			EncodedColorData[i*3+2] = ColorData[i].B;
		}

		TSharedPtr<ROSMessages::sensor_msgs::Image> ColorCamMsg(new ROSMessages::sensor_msgs::Image());
		ColorCamMsg->header.seq = HeaderSequence;
		ColorCamMsg->header.time = ROSTime;
		ColorCamMsg->header.frame_id = HeaderFrameID;
		ColorCamMsg->height = ImageHeight;
		ColorCamMsg->width = ImageWidth;
		ColorCamMsg->encoding = TEXT("rgb8");
		ColorCamMsg->is_bigendian = false;
		ColorCamMsg->step = ImageWidth * 3;
		ColorCamMsg->data = &EncodedColorData[0];
		ColorCamPub->Publish(ColorCamMsg);
	}
	
	HeaderSequence++;
}
