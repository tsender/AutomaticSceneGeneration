// Fill out your copyright notice in the Description page of Project Settings.


#include "Components/AnnotationComponent.h"
#include "Components/MeshComponent.h"
#include "AutoSceneGenLogging.h"

// Sets default values for this component's properties
UAnnotationComponent::UAnnotationComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	bWantsInitializeComponent = true;
}

// This code block executes before BeginPlay to initialize the AnnotationMID
void UAnnotationComponent::InitializeComponent() 
{
	Super::InitializeComponent();

	// Load materials
	FString AnnotationMaterialPath("/AutomaticSceneGeneration/Materials/M_AnnotationMaterial.M_AnnotationMaterial");
	FString DefaultMaterialPath("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial");

	AnnotationMaterialBase = LoadObject<UMaterialInterface>(nullptr, *AnnotationMaterialPath, nullptr, LOAD_None, nullptr);
	if (!AnnotationMaterialBase)
	{
		UE_LOG(LogASG, Error, TEXT("Cannot find material %s."), *AnnotationMaterialPath);
		return;
	}

	DefaultEngineMaterial = LoadObject<UMaterialInterface>(nullptr, *DefaultMaterialPath, nullptr, LOAD_None, nullptr);
	if (!DefaultEngineMaterial)
	{
		UE_LOG(LogASG, Error, TEXT("Cannot find material %s."), *DefaultMaterialPath);
		return;
	}

	// Make sure bUsedWithStaticLighting is set to true in Material Editor
	AnnotationMID = UMaterialInstanceDynamic::Create(AnnotationMaterialBase, this);
	if (!AnnotationMID)
	{
		UE_LOG(LogASG, Warning, TEXT("%s Annotation material instance dynamic could not be initialized."), *this->GetOwner()->GetName());
		return;
	}
	
	// Assume only one mesh component
	MeshComponent = Cast<UMeshComponent>(this->GetOwner()->GetComponentByClass(UMeshComponent::StaticClass()));
	if (!MeshComponent)
	{
		UE_LOG(LogASG, Warning, TEXT("Actor %s does not have a mesh component."), *this->GetOwner()->GetName());
		return;
	}
	DefaultMeshMaterials = MeshComponent->GetMaterials();
}

// Called when the game starts
void UAnnotationComponent::BeginPlay()
{
	Super::BeginPlay();

	// TODO: Change these from being hard-coded values to something that we can read from a JSON file
	if (GetOwner()->ActorHasTag(TEXT("sky")))
	{
		// We intentionally color the sky blue for the traversable color so the image looks more meaningful to a human
		AddAnnotationColor(EAnnotationColor::Traversable, FColor(0, 0, 255, 255)); // Blue
		AddAnnotationColor(EAnnotationColor::SemanticSegmentation, FColor(0, 0, 255, 255)); // Blue
		UE_LOG(LogASG, Display, TEXT("Found actor with tag 'sky'"));
	}

	if (GetOwner()->ActorHasTag(TEXT("ground_plane")))
	{
		AddAnnotationColor(EAnnotationColor::Traversable, FColor(255, 255, 255, 255)); // White
		AddAnnotationColor(EAnnotationColor::SemanticSegmentation, FColor(0, 255, 0, 255)); // Green
		UE_LOG(LogASG, Display, TEXT("Found actor with tag 'ground_plane'"));
	}
}

// Called every frame
void UAnnotationComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UAnnotationComponent::AddAnnotationColor(uint8 ColorID, FColor Color)
{
	LinearColorMap.Emplace(ColorID, Color.ReinterpretAsLinear());
}

void UAnnotationComponent::SetActiveMaterial(bool bAnnotation, uint8 ColorID)
{
	if (!MeshComponent)
	{
		UE_LOG(LogASG, Warning, TEXT("Mesh component is nullptr."));
		return;
	}

	if (!AnnotationMID)
	{
		UE_LOG(LogASG, Warning, TEXT("Annotation material instance dynamic is nullptr."));
		return;
	}
	
	if (bAnnotation && LinearColorMap.Contains(ColorID))
	{
		FLinearColor* Color = LinearColorMap.Find(ColorID);
		AnnotationMID->SetVectorParameterValue(FName("AnnotationColor"), *Color);
		for (int32 i = 0; i < DefaultMeshMaterials.Num(); i++)
		{
			MeshComponent->SetMaterial(i, AnnotationMID);
		}
		bAnnotationActive = true;
	}
	else
	{
		if (!bAnnotationActive) // Materials are already set to defaults
		{
			return;
		}

		for (int32 i = 0; i < DefaultMeshMaterials.Num(); i++)
		{
			MeshComponent->SetMaterial(i, DefaultMeshMaterials[i]);
		}
		bAnnotationActive = false;
	}
}
