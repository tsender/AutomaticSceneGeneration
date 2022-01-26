// Fill out your copyright notice in the Description page of Project Settings.


#include "Actors/StructuralSceneActor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/AnnotationComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"
#include "auto_scene_gen_logging.h"

AStructuralSceneActor::AStructuralSceneActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Static Mesh Component"));
	RootComponent = StaticMeshComponent;

	AnnotationComponent = CreateDefaultSubobject<UAnnotationComponent>(TEXT("Annotation Component"));
}

void AStructuralSceneActor::BeginPlay()
{
	Super::BeginPlay();

	// StaticMeshComponent->OnComponentHit.AddDynamic(this, &AStructuralSceneActor::OnHit);
	StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

	FVector Euler = GetActorRotation().Euler();
	FVector Location = GetActorLocation();
	FVector Scale = GetActorScale3D();

	Attributes = {1.f, Location.X, Location.Y, Euler.Z, Scale.X};
	bActive = true;

	// Update traversability status and annotation color
	if (bAlwaysTraversable)
	{
		bTraversable = true;
	}
	else
	{
		FVector Origin;
		FVector BoxExtent;
		GetActorBounds(true, Origin, BoxExtent);
		bTraversable = 2*BoxExtent.Z <= TraversableHeightThreshold;
	}
	UpdateTraversabilitySettings();
}

void AStructuralSceneActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AStructuralSceneActor::SetIDNumber(uint16 NewID) 
{
	IDNumber = NewID;
}

uint16 AStructuralSceneActor::GetIDNumber() const
{
	return IDNumber;
}

void AStructuralSceneActor::SetStructuralAttributes(TArray<float> NewAttributes) 
{
	if (NewAttributes.Num() != EStructuralSceneAttribute::Size)
	{
		UE_LOG(LogASG, Error, TEXT("New attribute array does not have length %i"), EStructuralSceneAttribute::Size);
		return;
	}
	Attributes = NewAttributes;

	// Update visibility
	bool Visibility = (bool)FMath::RoundToInt(Attributes[EStructuralSceneAttribute::Visibility]);
	SetActive(Visibility);

	// Set new location
	float NewX = Attributes[EStructuralSceneAttribute::X]; // [cm]
	float NewY = Attributes[EStructuralSceneAttribute::Y]; // [cm]
	float NewZ = GetActorLocation().Z;
	SetActorLocation(FVector(NewX, NewY, NewZ));

	// Set new rotation
	float NewYaw = Attributes[EStructuralSceneAttribute::Yaw]; // [deg]
	SetActorRotation(FRotator(0.f, NewYaw, 0.f));

	// Set new scale
	float NewScale = Attributes[EStructuralSceneAttribute::Scale];
	if (NewScale < 0.1f)
	{
		NewScale = 0.1f;
	}
	SetActorScale3D(FVector(NewScale, NewScale, NewScale));

	// Update traversability status and annotation color
	if (bAlwaysTraversable)
	{
		bTraversable = true;
	}
	else
	{
		FVector Origin;
		FVector BoxExtent;
		GetActorBounds(true, Origin, BoxExtent);
		bTraversable = 2*BoxExtent.Z <= TraversableHeightThreshold;
	}
	UpdateTraversabilitySettings();
}

void AStructuralSceneActor::SetActive(bool bNewActive) 
{
	if (bActive == bNewActive)
	{
		return;
	}

	bActive = bNewActive;
	SetActorHiddenInGame(!bActive);
	SetActorEnableCollision(bActive);
	SetActorTickEnabled(bActive);
}

bool AStructuralSceneActor::IsActive() const
{
	return bActive;
}

bool AStructuralSceneActor::IsTraversable() const
{
	return bTraversable;
}

void AStructuralSceneActor::UpdateTraversabilitySettings()
{	
	// Must manually set the traversable annotation material
	if (bTraversable)
	{
		AnnotationComponent->AddAnnotationColor(EAnnotationColor::Traversable, FLinearColor(1.f, 1.f, 1.f, 1.f));
		StaticMeshComponent->SetNotifyRigidBodyCollision(false);
		StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_Pawn, ECollisionResponse::ECR_Ignore);
		StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_PhysicsBody, ECollisionResponse::ECR_Ignore);
		StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_Vehicle, ECollisionResponse::ECR_Ignore);
		// UE_LOG(LogASG, Error, TEXT("SSA %s collision: none"), *GetName());
	}
	else
	{
		AnnotationComponent->AddAnnotationColor(EAnnotationColor::Traversable, FLinearColor(0.f, 0.f, 0.f, 1.f));
		StaticMeshComponent->SetNotifyRigidBodyCollision(true);
		StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		StaticMeshComponent->SetCollisionObjectType(ECollisionChannel::ECC_WorldStatic);
		// StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_WorldDynamic, ECollisionResponse::ECR_Block);
		StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_Pawn, ECollisionResponse::ECR_Block);
		StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_PhysicsBody, ECollisionResponse::ECR_Block);
		StaticMeshComponent->SetCollisionResponseToChannel(ECollisionChannel::ECC_Vehicle, ECollisionResponse::ECR_Block);
		// UE_LOG(LogASG, Error, TEXT("SSA %s collision: query and physics"), *GetName());
	}
}