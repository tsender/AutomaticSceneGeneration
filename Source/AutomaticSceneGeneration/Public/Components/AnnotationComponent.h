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

// This component can be used to change the color of the owning actor's mesh at runtime
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

	// Grab the latest default materials from the owning actor's mesh
	void UpdateDefaultMeshMaterials();

	/**
	 * Add an annotation color
	 * @param ColorID An ID for the color
	 * @param Color The annotation color to add
	 */
	void AddAnnotationColor(uint8 ColorID, FColor Color);
	
	/**
	 * Set which material is active in owning actor's mesh
	 * @param bAnnotation Indicates if the material to activate is one of the annotation colors
	 * @param ColorID The annotation color ID. Only used if bAnnotation is true
	 */
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
