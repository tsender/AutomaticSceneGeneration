// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "PIDDriveByWireComponent.generated.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class AUTOMATICSCENEGENERATION_API UPIDDriveByWireComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UPIDDriveByWireComponent();

protected: /****************************** UActorComponent Overrides ******************************/
	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	/****************************** UActorComponent Overrides ******************************/
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public: /****************************** UPIDDriveByWireComponent ******************************/
	bool IsManualDrive() const;

	float GetMaxManualDriveSpeed() const;

	float GetMaxSteeringAngle() const;

	void EnableDriveByWire(bool bEnable);

	void SetDesiredForwardVelocity(float NewVelocity);

	void SetDesiredSteeringAngle(float NewSteeringAngle);

	void SetHandbrakeInput(bool bEngaged);

private: /****************************** UPIDDriveByWireComponent ******************************/
	UPROPERTY()
	class AASGVehicle* Vehicle;
	
	// Indicates if we will be driving the vehicle via the keyboard
	UPROPERTY(EditAnywhere, Category = "Drive By Wire")
	bool bManualDrive = true;

	// Desired max manual speed [m/s]
	UPROPERTY(EditAnywhere, Category = "Drive By Wire")
	float MaxManualDriveSpeed = 5.;
	
	// Kp coefficient for PID throttle control
	UPROPERTY(EditAnywhere, Category = "Drive By Wire")
	float KpThrottle = 1.0;

	// Kd coefficient for PID throttle control
	UPROPERTY(EditAnywhere, Category = "Drive By Wire")
	float KdThrottle = 0.7;

	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	// ROS subscriber for vehicle pose (if we want to bypass physics calculations)
	UPROPERTY()
	class UTopic* BypassSub;

	// ROS subscriber for PhysX control inputs (longitudinal speed [m/s], steering angle [deg], and handbrake [true/false])
	UPROPERTY()
	class UTopic* PhysxControllerSub;

	UPROPERTY()
	class UWheeledVehicleMovementComponent* VehicleMovementComponent;

	bool bEnabled = false;
	bool bBypassController = false; // Updated based on what control messages arrive
	bool bProcessedBypassControlInput = true;
	
	float MaxSteeringAngle = 0.f;
	float DesiredVelocity = 0.f; // [m/s]
	float DesiredSteeringAngle = 0.f; // [deg]

	FVector BypassLocation;
	FQuat BypassQuat;

	void BypassControllerCB(TSharedPtr<FROSBaseMsg> Msg);

	void PhysxControllerCB(TSharedPtr<FROSBaseMsg> Msg);
	
	// Based on velocity
	void SetThrottleInput(float DeltaTime);
	
	void SetSteeringInput(float DeltaTime);

	void ProcessBypassControlInput();
};
