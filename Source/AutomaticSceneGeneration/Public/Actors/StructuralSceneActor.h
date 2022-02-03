// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "StructuralSceneActor.generated.h"

// Structural scene attributes (enum currently not used anymore)
enum EStructuralSceneAttribute
{
	Visibility,
	X,
	Y,
	Yaw,
	Scale,
	Size // This must be last
};

UCLASS()
class AUTOMATICSCENEGENERATION_API AStructuralSceneActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AStructuralSceneActor();

protected: /****************************** AActor Overrides ******************************/
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	/****************************** AActor Overrides ******************************/
	// Called every frame
	virtual void Tick(float DeltaTime) override;

public: /****************************** AStructuralSceneActor ******************************/
	UPROPERTY()
	class UAnnotationComponent* AnnotationComponent;

	void SetIDNumber(uint16 NewID);

	uint16 GetIDNumber() const;

	void SetStructuralAttributes(bool bVisibile, FVector NewLocation, FRotator NewRotation, float NewScale);

	// Use this to set if an actor is visible. We use the term "active" because several settings get modified.
	void SetActive(bool bNewActive);

	// Indicates if an actor is active/visible
	bool IsActive() const;

	bool IsTraversable() const;

private: /****************************** AStructuralSceneActor ******************************/
	UPROPERTY(EditAnywhere)
	class UStaticMeshComponent* StaticMeshComponent;

	// If the height in [cm] of the structural scene actor is below this threshold, then consider it traversable
	UPROPERTY(EditAnywhere)
	float TraversableHeightThreshold = 20; // [cm]

	// If a structural scene actor is always traversable (e.g. grass), then set this to true. This parameter takes priority over the TraversableHeightThreshold.
	UPROPERTY(EditAnywhere)
	bool bAlwaysTraversable = false;

	uint16 IDNumber; // Currently not used

	// Indicates if the actor is active (visible, can tick, etc.)
	bool bActive = true;

	bool bTraversable = false;

	void UpdateTraversabilitySettings();
};
