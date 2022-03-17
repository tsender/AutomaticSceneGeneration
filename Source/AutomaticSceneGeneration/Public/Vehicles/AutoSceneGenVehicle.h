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

	void SetDefaultResetInfo(FVector DefaultLocation, FRotator DefaultRotation);

	void ResetVehicle(FVector NewLocation, FRotator NewRotation);
	
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

	int32 CheckEnableTickNumber = 0;

	bool bEnabled = false;

	int32 TickNumber = 0;

	FVector ResetLocation;

	FRotator ResetRotation;

	float ResetTime = 0.f;

	// We currently assume we are on a flat ground plane
	float NominalVehicleZLocation = 0.f;

	// Number of structural scene actors hit
	int32 NumSSAHit = 0;

	UPROPERTY(EditAnywhere)
	FString VehicleName = "vehicle";

	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	UPROPERTY()
	class UTopic* EnableStatusPub;

	UPROPERTY()
	class UTopic* EnableStatusSub;

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
