// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "Triangle.generated.h"

// NOTE: Make sure to uncheck EnableWorldBoundsCheck in world settings

UCLASS()
class AUTOMATICSCENEGENERATION_API ATriangle : public AActor
{
	GENERATED_BODY()
	
public:
    ATriangle();

protected: /****************************** AActor Overrides ******************************/
	virtual void BeginPlay() override;

public:	/****************************** AActor Overrides ******************************/
	virtual void Tick(float DeltaTime) override;

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
    // The number of times we increase the mesh resolution
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