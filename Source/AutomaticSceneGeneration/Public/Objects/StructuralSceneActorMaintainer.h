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
	void Init(class UWorld* InWorld, TSubclassOf<class AStructuralSceneActor> InSSASubclass);

	void DestroyActors();

	void UpdateAttributes(TArray<bool> &NewVisibilities, TArray<FVector> &NewLocations, TArray<FRotator> &NewRotations, TArray<float> &NewScales);

private: /****************************** UStructuralSceneActorMaintainer ******************************/
	// Array of pointers to all maintained SSAs
	UPROPERTY()
	TArray<AStructuralSceneActor*> Ptrs;

	// The specific SSA subclass this UObject maintains
	TSubclassOf<AStructuralSceneActor> SSASubclass;

	// World context so we can spawn actors
	UWorld* World;
};
