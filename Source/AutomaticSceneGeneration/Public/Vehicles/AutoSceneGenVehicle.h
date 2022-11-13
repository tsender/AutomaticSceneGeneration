// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"
#include "auto_scene_gen_msgs/msg/OdometryWithoutCovariance.h"
#include "AutoSceneGenVehicle.generated.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UCLASS()
class AUTOMATICSCENEGENERATION_API AAutoSceneGenVehicle : public AWheeledVehicle
{
	GENERATED_BODY()

public:
	AAutoSceneGenVehicle();

protected: /****************************** AWheeledVehicle Overrides ******************************/
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public: /****************************** AWheeledVehicle Overrides ******************************/
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

public: /****************************** AAutoSceneGenVehicle ******************************/
	UPROPERTY(EditAnywhere)
	class UAnnotationComponent* AnnotationComponent;
	
	FString GetVehicleName() const;

	void SetWorldIsReadyFlag(bool bReady);

	bool IsEnabled() const;

	// Indicates if the vehicle is idling. Better to use GetIdleTime() to see how long the vehicle has been idling.
	bool IsVehicleIdling();

	// Indicates if the vehicle is stuck. Better to use GetStuckTime() to see how long the vehicle has been stuck, as this may return true as a vehicle begins to accelerate.
	bool IsVehicleStuck();

	// Returns the most recent consecutive amount of time the vehicle is found idling (near-zero motion with near-zero commanded velocity)
	float GetIdleTime() const;

	// Returns the most recent consecutive amount of time the vehicle is found stuck (near-zero motion but non-zero commanded velocity)
	float GetStuckTime() const;

	// Returns the amount of time that has passed since the vehicle received its first control command
	float GetTimeSinceFirstControl() const;

	void SetDefaultResetInfo(FVector DefaultLocation, FRotator DefaultRotation);

	/**
	 * Resets the vehicle
	 * @param NewLocation New reset location
	 * @param NewRotation New reset rotation
	 * @param bPreemptedDisable Indicates if vehicle was disabled preemptively (e.g., due to a forced reset by the AutoSceneGen worker)
	 */
	void ResetVehicle(FVector NewLocation, FRotator NewRotation, bool bPreemptedDisable = false);
	
	/**
	 * Resets the vehicle and writes the vehicle's trajectory into the appropriate field
	 * @param NewLocation New reset location
	 * @param NewRotation New reset rotation
	 * @param Trajectory Field to write the vehicle's last trajectory to
	 */
	void ResetVehicle(FVector NewLocation, FRotator NewRotation, TArray<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance> &Trajectory);

	float GetNominalVehicleZLocation();

	void GetVehicleTrajectory(TArray<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance> &Trajectory);

	int32 GetNumStructuralSceneActorsHit() const;

private: /****************************** AEvaluationVehicle ******************************/
	UPROPERTY(EditAnywhere)
	class UPIDDriveByWireComponent* DriveByWireComponent;

	UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UAudioComponent* EngineSoundComponent;

	UPROPERTY()
	TArray<class UBaseSensor*> Sensors;

	bool bWorldIsReady = false;

	FVector CheckEnableLocation;

	float CheckEnableTime = 0.;

	bool bEnabled = false;

	// Indicates if the vehicle was disabled preemptively (only applies if enabled is False)
	bool bPreempted = false;

	FVector ResetLocation;

	FRotator ResetRotation;

	float ResetTime = 0.f;

	// Motion under this linear speed [cm/s] is considered "not moving"
	UPROPERTY(EditAnywhere)
	float LinearMotionThreshold = 2.f;

	// Most recent consecutive amount of time the vehicle is found idling (near-zero motion with near-zero commanded velocity)
	float IdleTime = 0.f;

	// Most recent consecutive amount of time the vehicle is found stuck (near-zero motion but non-zero commanded velocity)
	float StuckTime = 0.f;

	float TimeSinceFirstControl = 0.f;

	// We currently assume we are on a flat ground plane
	float NominalVehicleZLocation = 0.f;

	// Number of structural scene actors hit
	int32 NumSSAHit = 0;

	UPROPERTY(EditAnywhere)
	FString VehicleName = "vehicle";

	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	UPROPERTY()
	class UTopic* VehicleStatusPub;

	TArray<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance> VehicleTrajectory;

	int32 HeaderSequence;

	int32 PathSequence;
	
	void DriveForward(float AxisValue);

	void SteerRight(float AxisValue);

	void OnHandbrakePressed();

	void OnHandbrakeReleased();

	void CheckIfReadyForEnable(float DeltaTime);

	void EnableSensors(bool bEnable);

	UFUNCTION()
	void OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, 
				FVector NormalImpulse, const FHitResult& Hit);
	
	// UFUNCTION()
	// void OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, 
	// 			UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult &SweepResult);
	
};
