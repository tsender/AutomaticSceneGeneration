// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/LandscapeAnnotationActor.h"
#include "Components/AnnotationComponent.h"
#include "Landscape.h"
#include "LandscapeComponent.h"
#include "Kismet/GameplayStatics.h"
#include "AutoSceneGenLogging.h"

// Sets default values
ALandscapeAnnotationActor::ALandscapeAnnotationActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALandscapeAnnotationActor::BeginPlay()
{
	Super::BeginPlay();

	TArray<AActor*> LandscapeActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ALandscape::StaticClass(), LandscapeActors);
	if (!LandscapeActors.Num())
	{
		UE_LOG(LogASG, Warning, TEXT("Could not find any landscape actors."));
		return;
	}

	Landscape = Cast<ALandscape>(LandscapeActors[0]);
	if (!Landscape)
	{
		UE_LOG(LogASG, Warning, TEXT("Could not cast landscape actor."));
		return;
	}
    Landscape->bUseDynamicMaterialInstance = true;

	// Load materials
	FString AnnotationMaterialPath("/Game/Materials/M_AnnotationMaterial.M_AnnotationMaterial");

	AnnotationMaterialBase = LoadObject<UMaterialInterface>(nullptr, *AnnotationMaterialPath, nullptr, LOAD_None, nullptr);
	if (!AnnotationMaterialBase)
	{
		UE_LOG(LogASG, Error, TEXT("Cannot find material %s."), *AnnotationMaterialPath);
		return;
	}

	// Make sure bUsedWithStaticLighting is set to true in Material Editor
	AnnotationMID = UMaterialInstanceDynamic::Create(AnnotationMaterialBase, this);
	if (!AnnotationMID)
	{
		UE_LOG(LogASG, Warning, TEXT("%s Annotation material instance dynamic could not be initialized."), *this->GetOwner()->GetName());
		return;
	}
	AnnotationMID->SetVectorParameterValue(FName("AnnotationColor"), FColor(0, 0, 0, 255));

	for (ULandscapeComponent* comp: Landscape->LandscapeComponents)
	{
		LandscapeComponentMaterials.Emplace(comp->GetMaterial(0));
        // UE_LOG(LogASG, Warning, TEXT("Comp has %i material instances"), comp->MaterialInstancesDynamic.Num())
        // comp->GetMaterialInstanceDynamic(0) = AnnotationMID;
        comp->MaterialInstancesDynamic.Add(AnnotationMID);
        comp->SetMaterial(0, AnnotationMID);
	}
	Landscape->bUseDynamicMaterialInstance = true;
	ColorMap.Emplace(EAnnotationColor::Traversable, FColor(0, 0, 0, 255));

    UE_LOG(LogASG, Warning, TEXT("Found num landscape comp materials: %i."), LandscapeComponentMaterials.Num());
}

// Called every frame
void ALandscapeAnnotationActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    SetActiveMaterial(true, EAnnotationColor::Traversable);
}

void ALandscapeAnnotationActor::SetActiveMaterial(bool bAnnotation, uint8 ColorID) 
{
	if (!AnnotationMID)
	{
		UE_LOG(LogASG, Warning, TEXT("Annotation material instance dynamic is nullptr."));
		return;
	}
	
	if (bAnnotation && ColorMap.Contains(ColorID))
	{
		FColor* Color = ColorMap.Find(ColorID);
		AnnotationMID->SetVectorParameterValue(FName("AnnotationColor"), *Color);
		for (ULandscapeComponent* comp: Landscape->LandscapeComponents)
		{
			comp->SetMaterial(0, AnnotationMID);
			// for(int32 i = 0; comp->MaterialInstances.Num(); i++)
			// {
			// 	comp->MarkRenderStateDirty();
			// 	FlushRenderingCommands();
			// 	comp->MaterialInstances[i] = (UMaterialInstanceConstant*)AnnotationMID;
			// 	comp->RecreateRenderState_Concurrent();
			// }
		}
		// Landscape->UpdateAllComponentMaterialInstances();
		// Landscape->LandscapeMaterial = AnnotationMID;
        // Landscape->EditorSetLandscapeMaterial(AnnotationMID);
		bAnnotationActive = true;
	}
	else
	{
		if (!bAnnotationActive) // Materials are already set to defaults
		{
			return;
		}

		for (int32 i=0; i < Landscape->LandscapeComponents.Num(); i++)
		{
			Landscape->LandscapeComponents[i]->MarkRenderStateDirty();
			FlushRenderingCommands();
			Landscape->LandscapeComponents[i]->SetMaterial(0, LandscapeComponentMaterials[i]);
		}
		// Landscape->UpdateAllComponentMaterialInstances();
		// Landscape->LandscapeMaterial = LandscapeComponentMaterials[0];
		bAnnotationActive = false;
	}
}
