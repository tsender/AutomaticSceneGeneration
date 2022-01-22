// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"
#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "ROSIntegration/Public/nav_msgs/Path.h"
#include "ASGVehicle.generated.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UCLASS()
class AUTOMATICSCENEGENERATION_API AASGVehicle : public AWheeledVehicle
{
	GENERATED_BODY()

public:
	AASGVehicle();

protected: /****************************** AWheeledVehicle Overrides ******************************/
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public: /****************************** AWheeledVehicle Overrides ******************************/
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

public: /****************************** AEvaluationVehicle ******************************/
	// UPROPERTY(EditAnywhere)
	// class UVehicleEvaluationComponent* EvaluationComponent;
	
	FString GetVehicleName() const;

	void SetWorldIsReadyFlag(bool bReady);

	bool IsEnabled() const;

	void SetDefaultResetInfo(FVector DefaultLocation, FRotator DefaultRotation);

	float ResetVehicle(FVector NewLocation, FRotator NewRotation);

	float GetNominalVehicleZLocation();

	void GetVehiclePath(class ROSMessages::nav_msgs::Path &Path);

private: /****************************** AEvaluationVehicle ******************************/
	UPROPERTY(EditAnywhere)
	class UPIDDriveByWireComponent* DriveByWireComponent;

	UPROPERTY(EditAnywhere)
	class UAnnotationComponent* AnnotationComponent;

	UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UAudioComponent* EngineSoundComponent;

	bool bWorldIsReady = false;

	FVector CheckEnableLocation;

	int32 CheckEnableTickNumber = 0;

	bool bEnabled = false;

	int32 TickNumber = 0;

	FVector ResetLocation;

	FRotator ResetRotation;

	float ResetTime = 0.f;

	float NominalVehicleZLocation = 0.f;

	UPROPERTY(EditAnywhere)
	FString VehicleName = "vehicle";

	UPROPERTY()
	class UROSIntegrationGameInstance* ROSInst;

	UPROPERTY()
	class UTopic* EnableStatusPub;

	UPROPERTY()
	class UTopic* EnableStatusSub;

	class ROSMessages::nav_msgs::Path VehiclePath;

	int32 HeaderSequence;

	int32 PathSequence;
	
	void DriveForward(float AxisValue);

	void SteerRight(float AxisValue);

	void OnHandbrakePressed();

	void OnHandbrakeReleased();

	void CheckIfReadyForEnable(float DeltaTime);

	UFUNCTION()
	void OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, 
				FVector NormalImpulse, const FHitResult& Hit);
	
	// UFUNCTION()
	// void OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, 
	// 			UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult &SweepResult);
	
};
