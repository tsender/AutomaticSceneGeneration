// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "StructuralSceneActor.generated.h"


/**
 * This class defines the core functionality for an AutoSceneGen structural scene actor.
 */
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

	/**
	 * Set/modify the actor's structural attributes all at once
	 * @param bVisible Indicates if the mesh is visible (traversability settings will be handled appropriately)
	 * @param bNewCastShadow Indicates if the mesh should cast a shadow
	 * @param Newlocation The new location
	 * @param NewRotation The new rotation
	 * @param NewScale The new scale factor (to be applied to each axis)
	 */
	void SetStructuralAttributes(bool bVisibile, bool bNewCastShadow, FVector NewLocation, FRotator NewRotation, float NewScale);

	// Use this to set if an actor is visible. We use the term "active" because several settings get modified.
	void SetActive(bool bNewActive);

	// Indicates if an actor is active/visible
	bool IsActive() const;

	void SetCastShadow(bool bNewCastShadow);

	void SetScale(float NewScale);

	bool IsTraversable() const;

private: /****************************** AStructuralSceneActor ******************************/
	UPROPERTY(EditAnywhere)
	class UStaticMeshComponent* StaticMeshComponent;

	UPROPERTY(EditAnywhere)
	// If the height in [cm] of the structural scene actor is below this threshold, then consider it traversable
	float TraversableHeightThreshold = 20;

	UPROPERTY(EditAnywhere)
	// If a structural scene actor is always traversable (e.g. grass), then set this to true. This parameter takes priority over the TraversableHeightThreshold.
	bool bAlwaysTraversable = false;

	uint16 IDNumber; // Currently not used

	// Indicates if the actor is active (visible, can tick, etc.)
	bool bActive = true;

	bool bTraversable = false;

	float DefaultHeight;

	// Updates the actor's traversability annotation color and its collision settings
	void UpdateTraversabilitySettings();
};
