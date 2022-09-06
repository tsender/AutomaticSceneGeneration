// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "Terrain.generated.h"

// NOTE: 
// 1) Set World Settings --> world --> navigation system config --> NullNavSysConfig to disable 
// "LogNavigationDirtyArea: Warning: Skipping dirty area creation because of empty bounds" warning (only affects nav system used by AI)
// 2) Set World Settings --> Lightmass --> check Force No Precomputed Lighting


UCLASS()
class AUTOMATICSCENEGENERATION_API ATerrain : public AActor
{
	GENERATED_BODY()
	
public:
    ATerrain();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

    // Manipulate the vertices that are close to the impact point vector parameter
    UFUNCTION(BlueprintCallable)
    void AlterTerrain(FVector ImpactPoint);

private:
    UPROPERTY(EditAnywhere)
    class UProceduralMeshComponent* TerrainMesh;

    UPROPERTY(EditAnywhere)
    TArray<FVector> Vertices;

    UPROPERTY(EditAnywhere)
    TArray<int> Triangles;

    UPROPERTY(EditAnywhere)
    TArray<FVector> Normals;

    TArray<FVector2D> UV0;

    UPROPERTY(EditAnywhere)
    TArray<FLinearColor> VertexColors;

    TArray<FColor> UpVertexColors;
    TArray<FProcMeshTangent> Tangents;

    UPROPERTY(EditAnywhere)
    TArray<int> NewTriangles;

    UPROPERTY(EditAnywhere)
    TArray<FVector> NewVertices;

    // Triangle points a,b,c
    int IdxA = 0;
    int IdxB = 1;
    int IdxC = 2;

    UPROPERTY(EditAnywhere)
    // The number of times we increase the mesh resolution (each original face will have 2^Recursion number faces)
    int Recursions;

    // Keeps a record of all vertices
    TArray<FVector> VertexDict;

    // Keeps a record of all triangle idxs
    TArray<int> IndicesDict;

    // Temporarily store the triangle idxs and end points so we can build a new index list when subdividing a given face
    int i_a, i_b, i_c, i_ab, i_bc, i_ca;

    void GenerateMesh();

    void Subdivide(int a, int b, int c);
};