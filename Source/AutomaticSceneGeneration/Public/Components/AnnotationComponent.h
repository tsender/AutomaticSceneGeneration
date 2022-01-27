// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AnnotationComponent.generated.h"

// Supported types of annotation colors
enum EAnnotationColor
{
	Traversable,
	SemanticSegmentation,
	InstanceSegmentation
};

// This component can be used to change the color of the owning actor's static mesh at runtime
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class AUTOMATICSCENEGENERATION_API UAnnotationComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UAnnotationComponent();

	virtual void InitializeComponent() override;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void AddAnnotationColor(uint8 ColorID, FColor Color);
	void SetActiveMaterial(bool bAnnotation, uint8 ColorID);

private:	
	UPROPERTY()
	UMaterialInterface* DefaultEngineMaterial;
	
	UPROPERTY()
	UMaterialInterface* AnnotationMaterialBase;
	
	UPROPERTY()
	UMaterialInstanceDynamic* AnnotationMID;

	UPROPERTY()
	TArray<UMaterialInterface*> DefaultMeshMaterials;

	UPROPERTY()
	TMap<uint8, FLinearColor> LinearColorMap;

	UPROPERTY()
	class UMeshComponent* MeshComponent;

	bool bAnnotationActive = false;	
};
