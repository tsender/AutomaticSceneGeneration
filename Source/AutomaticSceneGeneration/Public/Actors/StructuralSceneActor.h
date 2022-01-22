// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "StructuralSceneActor.generated.h"

// ODD Static structural attributes
enum EStructuralSceneAttribute
{
	Visibility,
	X,
	Y,
	Yaw,
	Scale,
	Size // This must be last
};

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
	void SetIDNumber(uint16 NewID);

	uint16 GetIDNumber() const;

	void SetStructuralAttributes(TArray<float> NewAttributes);

	void SetActive(bool bNewActive);

	bool IsActive() const;

	bool IsTraversable() const;

private: /****************************** AStructuralSceneActor ******************************/
	UPROPERTY(EditAnywhere)
	class UStaticMeshComponent* StaticMeshComponent;

	UPROPERTY()
	class UAnnotationComponent* AnnotationComponent;

	UPROPERTY(EditAnywhere)
	float TraversableHeightThreshold = 20; // [cm]

	TArray<float> Attributes;

	uint16 IDNumber;

	bool bActive = true;

	bool bTraversable = false;

	void UpdateTraversabilitySettings();
};
