// Fill out your copyright notice in the Description page of Project Settings.


#include "Objects/StructuralSceneActorMaintainer.h"
#include "Actors/StructuralSceneActor.h"

UStructuralSceneActorMaintainer::UStructuralSceneActorMaintainer()
{

}

void UStructuralSceneActorMaintainer::BeginDestroy()
{
    Super::BeginDestroy();
    DestroyActors();
}

void UStructuralSceneActorMaintainer::Init(UWorld* InWorld, TSubclassOf<AStructuralSceneActor> InSSASubclass)
{
    World = InWorld;
    SSASubclass = InSSASubclass;
}

void UStructuralSceneActorMaintainer::DestroyActors()
{
    for (AStructuralSceneActor* Actor : Ptrs)
    {
        Actor->SetActive(false);
        Actor->Destroy();
    }
    Ptrs.Empty();
}

void UStructuralSceneActorMaintainer::UpdateAttributes(TArray<bool> &NewVisibilities, TArray<bool> &NewCastShadows, TArray<FVector> &NewLocations, TArray<FRotator> &NewRotations, TArray<float> &NewScales)
{
    int32 NumRequestedInstances = NewVisibilities.Num();
    if (NewCastShadows.Num() != NumRequestedInstances) return;
    if (NewLocations.Num() != NumRequestedInstances) return;
    if (NewRotations.Num() != NumRequestedInstances) return;
    if (NewScales.Num() != NumRequestedInstances) return;

    if (NumRequestedInstances == 0)
    {
        DestroyActors();
    }
    else
    {
        SetNumActors(NumRequestedInstances, true);

        // Update SSA parameters
        for (int32 i = 0; i < Ptrs.Num(); i++)
        {
            Ptrs[i]->SetStructuralAttributes(NewVisibilities[i], NewCastShadows[i], NewLocations[i], NewRotations[i], NewScales[i]);
        }
    }
}

void UStructuralSceneActorMaintainer::SetNumActors(int32 NumActorsToMaintain, bool bMakeNewActorsVisible)
{
    while (Ptrs.Num() != NumActorsToMaintain)
    {
        if (NumActorsToMaintain < Ptrs.Num()) // Destroy actors
        {
            Ptrs[Ptrs.Num()-1]->SetActive(false); // Set to invisible in case it does not get gc'd immediately
            Ptrs[Ptrs.Num()-1]->Destroy();
            Ptrs.RemoveAt(Ptrs.Num()-1);
        }
        else if (NumActorsToMaintain > Ptrs.Num()) // Add actors
        {
            // Spawn at arbitrary location and rotation, will be updated later
            AStructuralSceneActor* Actor = World->SpawnActor<AStructuralSceneActor>(SSASubclass, FVector(0,0,0), FRotator(0,0,0));
            Actor->SetActive(bMakeNewActorsVisible);
            Ptrs.Emplace(Actor);
        }
    }
}

void UStructuralSceneActorMaintainer::SetAllActorsInvisible()
{
    for (AStructuralSceneActor* Actor : Ptrs)
        Actor->SetActive(false);
}

AStructuralSceneActor* UStructuralSceneActorMaintainer::GetActor(int32 Idx) const
{
    return (0 <= Idx && Idx < Ptrs.Num()) ? Ptrs[Idx] : nullptr;
}