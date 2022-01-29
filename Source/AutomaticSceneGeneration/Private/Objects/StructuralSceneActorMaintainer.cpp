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
        Actor->Destroy();
    }
    Ptrs.Empty();
}

void UStructuralSceneActorMaintainer::UpdateAttributes(TArray<bool> &NewVisibilities, TArray<FVector> &NewLocations, TArray<FRotator> &NewRotations, TArray<float> &NewScales)
{
    int32 NumRequestedInstances = NewVisibilities.Num();
    if (NumRequestedInstances == 0)
    {
        DestroyActors();
    }
    else
    {
        // Update number of SSA instances in the world
        while (Ptrs.Num() != NumRequestedInstances)
        {
            if (NumRequestedInstances < Ptrs.Num())
            {
                Ptrs[Ptrs.Num()-1]->Destroy();
                Ptrs.RemoveAt(Ptrs.Num()-1);
            }
            else if (NumRequestedInstances > Ptrs.Num())
            {
                // Spawn at arbitrary location and rotation, will be updated later
                AStructuralSceneActor* Actor = World->SpawnActor<AStructuralSceneActor>(SSASubclass, FVector(0,0,0), FRotator(0,0,0));
                Ptrs.Emplace(Actor);
            }
        }

        // Update SSA parameters
        for (int32 i = 0; i < Ptrs.Num(); i++)
        {
            Ptrs[i]->SetStructuralAttributes(NewVisibilities[i], NewLocations[i], NewRotations[i], NewScales[i]);
        }
    }
}