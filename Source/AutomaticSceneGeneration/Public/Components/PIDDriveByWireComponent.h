// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PIDDriveByWireComponent.generated.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

/**
 * NOTE: This controller uses [m/s] instead of [cm/s] when specifying control velocities.
 * For the time being, this class only implements a PD controller.
 */
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

	bool ReceivedFirstControlInput() const;

	float GetMaxManualDriveSpeed() const;

	float GetMaxSteeringAngle() const;

	float GetDesiredVelocity() const;

	float GetDesiredSteeringAngle() const;

	void EnableDriveByWire(bool bEnable);

	void SetDesiredForwardVelocity(float NewVelocity);

	void SetDesiredSteeringAngle(float NewSteeringAngle);

	void SetHandbrakeInput(bool bEngaged);

private: /****************************** UPIDDriveByWireComponent ******************************/
	UPROPERTY()
	class AAutoSceneGenVehicle* Vehicle;
	
	UPROPERTY(EditAnywhere, Category = "PID Drive By Wire")
	// Indicates if we will be driving the vehicle via the keyboard
	bool bManualDrive = false;

	UPROPERTY(EditAnywhere, Category = "PID Drive By Wire")
	// Desired max manual speed [cm/s]
	float MaxManualDriveSpeed = 500.;
	
	UPROPERTY(EditAnywhere, Category = "PID Drive By Wire")
	// Kp coefficient for PID throttle control. NOTE: Control velocity is in [cm/s]
	float KpThrottle = 0.0002;

	UPROPERTY(EditAnywhere, Category = "PID Drive By Wire")
	// Kd coefficient for PID throttle control. NOTE: Control velocity is in [cm/s]
	float KdThrottle = 0.0007;

	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	UPROPERTY()
	// ROS subscriber for vehicle pose (if we want to bypass physics calculations)
	class UTopic* BypassSub;

	UPROPERTY()
	// ROS subscriber for PhysX control inputs (longitudinal speed [m/s], steering angle [deg], and handbrake [true/false])
	class UTopic* PhysxControllerSub;

	UPROPERTY()
	class UWheeledVehicleMovementComponent* VehicleMovementComponent;

	bool bEnabled = false;

	bool bReceivedFirstControlInput = false;

	bool bBypassController = false; // Updated based on what control messages arrive

	bool bProcessedBypassControlInput = true;
	
	float MaxSteeringAngle = 0.f;

	float DesiredVelocity = 0.f; // [m/s]

	float DesiredSteeringAngle = 0.f; // [deg]

	FVector BypassLocation;

	FQuat BypassQuat;

	/**
	 * ROS callback for explicitly moving the vehicle to a new pose. Use this if you wish to bypass the actual controller and simply want to update the vehicle's pose manually.
	 * @param Msg The geometry_msgs/Pose message with the desired pose
	 */
	void BypassControllerCB(TSharedPtr<class FROSBaseMsg> Msg);

	/**
	 * ROS callback for sending PhysX control commands to the vehicle
	 * @param Msg The vehicle_msgs/PhysXControl message with the desired control commands
	 */
	void PhysxControllerCB(TSharedPtr<class FROSBaseMsg> Msg);
	
	// Based on velocity
	void SetThrottleInput(float DeltaTime);
	
	void SetSteeringInput(float DeltaTime);

	void ProcessBypassControlInput();
};
