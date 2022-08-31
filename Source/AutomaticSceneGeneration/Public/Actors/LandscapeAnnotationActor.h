// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LandscapeAnnotationActor.generated.h"

/**
 * Commented some "#if WITH_EDITOR" macros in these files: Landscape.cpp, LandscapeEdit.cpp, LandscapeComponent.h
 * Search for lines with "// UNCOMMENT"
*/

UCLASS()
class AUTOMATICSCENEGENERATION_API ALandscapeAnnotationActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ALandscapeAnnotationActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void SetActiveMaterial(bool bAnnotation, uint8 ColorID);

private:
	UPROPERTY()
	class ALandscape* Landscape;

	UPROPERTY()
	UMaterialInterface* AnnotationMaterialBase;

	UPROPERTY()
	TArray<UMaterialInterface*> LandscapeComponentMaterials;

	UPROPERTY()
	UMaterialInstanceDynamic* AnnotationMID;

	UPROPERTY()
	TMap<uint8, FColor> ColorMap;

	bool bAnnotationActive = false;	
};
