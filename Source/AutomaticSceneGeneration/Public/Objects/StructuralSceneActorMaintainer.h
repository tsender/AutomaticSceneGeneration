// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "StructuralSceneActorMaintainer.generated.h"

/**
 * This struct maintains all SSAs of a given subclass. It can add actors with specified parameters and also remove actors.
 * All calculations for the actor parameters should be done before passing them to this struct.
 */
UCLASS()
class AUTOMATICSCENEGENERATION_API UStructuralSceneActorMaintainer : public UObject
{
	GENERATED_BODY()

public: /****************************** UObject Overrides ******************************/
	UStructuralSceneActorMaintainer();

	virtual void BeginDestroy() override;

public: /****************************** UStructuralSceneActorMaintainer ******************************/
	/**
	 * Initialize the maintainer
	 * @param InWorld Pointer to the world context
	 * @param InSSASubclass The specific subclass of AStructuralSceneActors that this maintainer will manage
	 */
	void Init(class UWorld* InWorld, TSubclassOf<class AStructuralSceneActor> InSSASubclass);

	// Destroy all managed structural scene actors
	void DestroyActors();

	/**
	 * Update the structural attributes for all managed actors. All arrays must be of the same length, or nothing will be updated
	 * @param NewVisibilities Array of bools indicating which actors are visible
	 * @param NewCastShadows Array of bools indicating which actors can cast shadows
	 * @param NewLocations Array of new locations for each actor
	 * @param NewRotations Array of new rotations for each actor
	 * @param NewScales Array of new scale factors for each actor
	 */
	void UpdateAttributes(TArray<bool> &NewVisibilities, TArray<bool> &NewCastShadows, TArray<FVector> &NewLocations, TArray<FRotator> &NewRotations, TArray<float> &NewScales);

	/**
	 * Set the number of actors to maintain
	 * @param NumActorsToMaintain The number of actors to maintain
	 * @param bMakeNewActorsVisible Indicate if newly added actors should be visible
	 */
	void SetNumActors(int32 NumActorsToMaintain, bool bMakeNewActorsVisible);

	// Set all maintained actors to be invisible
	void SetAllActorsInvisible();

	/**
	 * Get a pointer to the AStructuralSceneActor at the specified index
	 * @param Idx Index of actor to get a pointer to
	 */
	AStructuralSceneActor* GetActor(int32 Idx) const;

private: /****************************** UStructuralSceneActorMaintainer ******************************/
	// Array of pointers to all maintained SSAs
	UPROPERTY()
	TArray<AStructuralSceneActor*> Ptrs;

	// The specific SSA subclass this UObject maintains
	TSubclassOf<AStructuralSceneActor> SSASubclass;

	// World context so we can spawn actors
	UWorld* World;
};
