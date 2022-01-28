// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "auto_scene_gen_msgs/StructuralSceneAttributes.h"
#include "StructuralSceneActorMaintainer.generated.h"

// This struct is used to temporarily store the new attributes for the SSAs of a given subclass
struct FStructuralSceneActorAttr
{
	// Stores the incoming concatenated attribute array from the RunScenario request
	TArray<float> ConcatAttrArray;
	
	TArray<bool> Visibilities;
	TArray<FVector> Locations;
	TArray<FRotator> Rotations;
	TArray<float> Scales;
	
	// The path name of the SSA subclass that this struct maintains
	FString SSAPathName;

	int32 NumSSAs;

	FStructuralSceneActorAttr(/*FString InSSAPathName*/)
	{
		// SSAPathName = InSSAPathName;
	}

	~FStructuralSceneActorAttr() 
	{
		ConcatAttrArray.Empty();
		Visibilities.Empty();
		Locations.Empty();
		Rotations.Empty();
		Scales.Empty();
	}

	void StoreAttributeArray(int32 NewNumSSAs, TArray<float> &NewConcatAttrArray)
	{
		NumSSAs = NewNumSSAs;
		ConcatAttrArray = NewConcatAttrArray;
	}

	void StoreTArrays(TArray<bool> &NewVisibilities, TArray<FVector> &NewLocations, TArray<FRotator> &NewRotations, TArray<float> &NewScales)
	{
		Visibilities = NewVisibilities;
		Locations = NewLocations;
		Rotations = NewRotations;
		Scales = NewScales;
	}
};

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
	void Init(class UWorld* InWorld, TSubclassOf<class AStructuralSceneActor> InSSASubclass);

	void DestroyActors();

	void UpdateAttributes(TArray<bool> &NewVisibilities, TArray<FVector> &NewLocations, TArray<FRotator> &NewRotations, TArray<float> &NewScales);

private: /****************************** UStructuralSceneActorMaintainer ******************************/
	// Array of pointers to all maintained SSAs
	UPROPERTY()
	TArray<AStructuralSceneActor*> Ptrs;

	// The specific SSA subclass this UObject maintains
	TSubclassOf<AStructuralSceneActor> SSASubclass;

	// The path name of the SSA subclass that this struct maintains
	FString SSAPathName;

	// World context so we can spawn actors
	UWorld* World;
};
