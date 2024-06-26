// Fill out your copyright notice in the Description page of Project Settings.


#include "Sensors/CompleteCameraSensor.h"
#include "Sensors/ColorCameraSensor.h"
#include "Sensors/DepthCameraSensor.h"
#include "Sensors/SegmentationCameraSensor.h"
#include "Components/AnnotationComponent.h"
#include "Actors/AutoSceneGenWorker.h"
#include "Vehicles/AutoSceneGenVehicle.h"
#include "Kismet/GameplayStatics.h"
#include "Json/Public/Serialization/JsonSerializer.h"
#include "AutoSceneGenLogging.h"

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

	HeaderSequence = 1;

	EncodedColorData.resize(ImageWidth*ImageHeight*3);
	EncodedDepthData.resize(ImageWidth*ImageHeight*4);
	EncodedTravData.resize(ImageWidth*ImageHeight*3);
	EncodedSegData.resize(ImageWidth*ImageHeight*3);

	ColorCamera->InitTextureTarget(ImageWidth, ImageHeight, FieldOfView);
	DepthCamera->InitTextureTarget(ImageWidth, ImageHeight, FieldOfView);
	TravCamera->InitTextureTarget(ImageWidth, ImageHeight, FieldOfView);
	SegCamera->InitTextureTarget(ImageWidth, ImageHeight, FieldOfView);

	ColorCamera->SetSaveImages(bSaveImagesToDisk);
	TravCamera->SetSaveImages(bSaveImagesToDisk);
	SegCamera->SetSaveImages(bSaveImagesToDisk);

	ColorCamera->PostProcessSettings = ColorCameraPostProcessSettings;

	CameraFolder = FPaths::ProjectUserDir() + FString::Printf(TEXT("%s/%s/"), *SaveFolder, *SensorName);
	FrameNumberPath = CameraFolder + FString("FrameNumber.json");
	ColorCamera->SetSavePrefix(CameraFolder + FString("color/color"));
	TravCamera->SetSavePrefix(CameraFolder + FString("trav/trav"));
	SegCamera->SetSavePrefix(CameraFolder + FString("seg/seg"));

	// Load next valid frame number
	if (bSaveImagesToDisk && FPaths::FileExists(FrameNumberPath))
	{
		FString JsonString;
		FFileHelper::LoadFileToString(JsonString, *FrameNumberPath);
		TSharedPtr<FJsonValue> JsonValue;
		TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);

		if (FJsonSerializer::Deserialize(Reader, JsonValue))
		{
			int32 FrameNumber = JsonValue->AsObject()->GetIntegerField("FrameNumber");
			UE_LOG(LogASG, Display, TEXT("Camera sensor '%s': Setting next frame number to %i"), *SensorName, FrameNumber);
			ColorCamera->SetFrameNumber(FrameNumber);
			TravCamera->SetFrameNumber(FrameNumber);
			SegCamera->SetFrameNumber(FrameNumber);
		}
	}

	ROSInst = Cast<UROSIntegrationGameInstance>(GetOwner()->GetGameInstance());
	if (ROSInst && ROSInst->bConnectToROS)
	{
		// Create topic prefix
		FString TopicPrefix = FString("/sensors/") + SensorName;
		HeaderFrameID = SensorName;

		AAutoSceneGenVehicle* OwningVehicle = Cast<AAutoSceneGenVehicle>(GetOwner());
		if (OwningVehicle)
		{
			TopicPrefix = FString::Printf(TEXT("/%s"), *OwningVehicle->GetVehicleName()) + TopicPrefix;
			HeaderFrameID = FString::Printf(TEXT("/%s/"), *OwningVehicle->GetVehicleName()) + SensorName;
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
		FString ColorTopic = TopicPrefix + FString("/color_image");
		ColorCamPub->Init(ROSInst->GetROSConnectionFromID(ROSBridgeServerID), ColorTopic, TEXT("sensor_msgs/Image"));
		ColorCamPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *ColorTopic);

		if (bEnableDepthCam)
		{
			DepthCamPub = NewObject<UTopic>(UTopic::StaticClass());
			FString DepthTopic = TopicPrefix + FString("/depth_image");
			DepthCamPub->Init(ROSInst->GetROSConnectionFromID(ROSBridgeServerID), DepthTopic, TEXT("sensor_msgs/Image"));
			DepthCamPub->Advertise();
			UE_LOG(LogASG, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *DepthTopic);
		}
		
		if (bEnableTravCam)
		{
			TravCamPub = NewObject<UTopic>(UTopic::StaticClass());
			FString TravTopic = TopicPrefix + FString("/trav_image");
			TravCamPub->Init(ROSInst->GetROSConnectionFromID(ROSBridgeServerID), TravTopic, TEXT("sensor_msgs/Image"));
			TravCamPub->Advertise();
			UE_LOG(LogASG, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *TravTopic);
		}

		if (bEnableSegCam)
		{
			SegCamPub = NewObject<UTopic>(UTopic::StaticClass());
			FString SegTopic = TopicPrefix + FString("/seg_image");
			SegCamPub->Init(ROSInst->GetROSConnectionFromID(ROSBridgeServerID), SegTopic, TEXT("sensor_msgs/Image"));
			SegCamPub->Advertise();
			UE_LOG(LogASG, Display, TEXT("Initialized camera sensor ROS publisher: %s"), *SegTopic);
		}
	}

	FTimerHandle TimerHandle;
	GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &UCompleteCameraSensor::TickSensor, 1./FrameRate, true);
}

void UCompleteCameraSensor::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
	Super::EndPlay(EndPlayReason);

	EncodedColorData.clear();
	EncodedDepthData.clear();
	EncodedTravData.clear();
	EncodedSegData.clear();

	// Save next valid frame number
	if (bSaveImagesToDisk)
	{
		int32 NextFrameNumber = ColorCamera->GetFrameNumber();
		TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);
		JsonObject->SetNumberField("FrameNumber", NextFrameNumber);

		FString JsonString;
		TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);

		if (FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer))
		{
			FFileHelper::SaveStringToFile(JsonString, *FrameNumberPath);
			UE_LOG(LogASG, Display, TEXT("Camera sensor '%s': Saving next frame number to %i in JSON file"), *SensorName, NextFrameNumber);
		}
	}
}

// Called every frame
void UCompleteCameraSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

// Primary tick function for the camera sensor. Encode all message data in native byte ordering to minimize runtime cost.
void UCompleteCameraSensor::TickSensor()
{
	if (!bEnabled) return;

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
				
				// Encode float32 as 4 bytes with native byte ordering
				std::memcpy(&EncodedDepthData[i*4], &d, sizeof(float)); // memcpy order: dest, src, size
			}

			TSharedPtr<ROSMessages::sensor_msgs::Image> DepthCamMsg(new ROSMessages::sensor_msgs::Image());
			DepthCamMsg->header.seq = HeaderSequence;
			DepthCamMsg->header.time = ROSTime;
			DepthCamMsg->header.frame_id = HeaderFrameID;
			DepthCamMsg->height = ImageHeight;
			DepthCamMsg->width = ImageWidth;
			DepthCamMsg->encoding = TEXT("32FC1"); // Each pixel encodes the float32 depth as 4 uint8 bytes
#if PLATFORM_LITTLE_ENDIAN
			DepthCamMsg->is_bigendian = false;
#else
			DepthCamMsg->is_bigendian = true;
#endif
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
