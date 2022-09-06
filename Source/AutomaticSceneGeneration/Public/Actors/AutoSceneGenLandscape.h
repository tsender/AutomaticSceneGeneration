// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "AutoSceneGenLandscape.generated.h"

// NOTE: 
// 1) Set World Settings --> world --> navigation system config --> NullNavSysConfig to disable 
// "LogNavigationDirtyArea: Warning: Skipping dirty area creation because of empty bounds" warning (only affects nav system used by AI)
// 2) Set World Settings --> Lightmass --> check Force No Precomputed Lighting

// TODO: Split landscape into sections so that sculpt functions can more quickly search through vertices within a section rather than the entire landscape

enum LandscapeFalloff
{
    Linear,
    Smooth
};

UCLASS()
class AUTOMATICSCENEGENERATION_API AAutoSceneGenLandscape : public AActor
{
	GENERATED_BODY()
	
public:
    AAutoSceneGenLandscape();

protected: /****************************** AActor Overrides ******************************/
	virtual void BeginPlay() override;

public: /****************************** AActor Overrides ******************************/
	virtual void Tick(float DeltaTime) override;

public: /****************************** AAutoSceneGenLandscape ******************************/
    /**
     * Create the base (flat) square mesh with the specified parameters
     * @param Location The center of the landscape
     * @param Size The side-length in [cm] of the landscape along the X and Y dimensions (this is a square landscape)
     * @param NumSubdivisions The number of times the two base triangles should be subdivided. The landscape will have 2^NumSubdivisions triangles along each edge.
     *      Each vertex in the mesh will be spaced Size/(2^NumSubdivisions) [cm] apart in a grid.
     */
    bool CreateBaseMesh(FVector Location, float Size, int Subdivisions);
    
    // UFUNCTION(BlueprintCallable)
    // // Manipulate the vertices that are close to the impact point vector parameter
    // void AlterTerrain(FVector ImpactPoint);

    void GetVertexIndicesWithinRadius(FVector P, float Radius, TArray<int32> &Indices);

    void GetVertexMapCoordinatesWithinRadius(FVector P, float Radius, TArray<FIntPoint> &VertexMapCoords);

    /**
     * Add a 2D bell-shaped curve to the existing landscape (can be up- or downward facing)
     * @param Mean The center of the gaussian (only the XY coordinates are used)
     * @param Stddev The standard deviation in [cm], applies to all directions (we will only modify vertices within 5 stddev of the center)
     * @param Height The Z coordinate of the Gaussian (i.e., the maximum amount of change in the Z direction in [cm])
     */
    void LandscapeEditSculptGaussian(FVector Mean, float Stddev, float Height);

    /**
     * Raise height data to the landscape within the specified circular region
     */
    // void LandscapeEditSculptCylinder(float Center, float Radius);

    // void LandscapeEditAddRamp();

    // void LandscapeEditFlatten();

private: /****************************** AAutoSceneGenLandscape ******************************/    
    UPROPERTY(EditAnywhere)
    class UProceduralMeshComponent* TerrainMesh;

    TArray<FVector> Vertices;

    TArray<int32> Triangles;

    TArray<FVector> Normals;

    TArray<FVector2D> UV0;

    TArray<FLinearColor> VertexColors;

    TArray<FColor> UpVertexColors;

    TArray<FProcMeshTangent> Tangents;

    // The side-length in [cm] of the landscape along the X and Y dimensions
    int LandscapeSize;

    // Horizontal distance in [cm] between adjacent vertices in the landscape mesh
    float VertexSeparation;
    
    FVector LowerLeftCorner;

    // The number of times the two base triangles should be subdivided. The landscape will have 2^NumSubdivisions triangles along each edge.
    // Each vertex in the mesh will be spaced Size/(2^NumSubdivisions) [cm] apart in a grid.
    int NumSubdivisions;

    // The vertex grid map is a XY grid that maps a vertex's (x,y) integer coordinate to a normalized grid with 1 unit spacing.
    // The lower left is (0,0) and the upper right is (N,N), following Unreal's left-hand coordinate system (X points up, Y points right).
    // This allows us to more quickly lookup the index in the Vertices buffer that corresponds to a certain coordinate.
    TArray<TArray<int32> > VertexGridMap;

    /**
     * Subdivide triangle ABC (clockwise order) into 4 smaller triangles
     * @param i_a Index of vertex A
     * @param i_b Index of vertex B
     * @param i_c Index of vertex C
     * @param MeshVertices Original list of all vertex positions in current mesh
     * @param AddedVertices Stores the vertices added from the subdivision process (speeds up the process for checking if a vertex has already been added)
     * @param AddedIndices Stores the vertex indices added from the subdivision process (speeds up the process for checking if a vertex has already been added)
     * @param NewVertices The new list of vertex positions for the mesh
     * @param NewTriangles The new index buffer indicating which vertices make up each triangle. Length is a multiple of 3.
     */
    void SubdivideTriangle(int32 i_a, int32 i_b, int32 i_c, const TArray<FVector> &MeshVertices, TArray<FVector> &AddedVertices, TArray<int32> &AddedIndices, TArray<FVector> &NewVertices, TArray<int32> &NewTriangles);

    /**
     * Subdivide the provided mesh by the number of specified subdivisions
     * @param Subdivisions Number of times to subdivide each face
     * @param Vertices The list of vertex positions for the mesh (when done, will contain a new list of vertex positions)
     * @param Triangles The index buffer indicating which vertices make up each triangle (when done, will contain the new index buffer). Length is a multiple of 3.
     */
    void SubdivideMesh(int32 Subdivisions, TArray<FVector> &MeshVertices, TArray<int32> &MeshTriangles);

    /**
     * Compute brush strength based on a linear falloff. Max strength = 1 and occurs when Distance <= EffectiveRadius. Min strength = 0 and occurs when Distance >= BrushRadius.
     * @param BrushRadius The brush's radius in [cm]
     * @param EffectiveRadius The effective brush radius in [cm] (region where brush strength = 1)
     * @param Distance The distance from the center of the brush in [cm]
     */
    float CalculateLinearBrushStrength(float BrushRadius, float EffectiveRadius, float Distance);

    /**
     * Compute brush strength based on a smooth falloff. Max strength = 1 and occurs when Distance <= EffectiveRadius. Min strength = 0 and occurs when Distance >= BrushRadius.
     * Falloff function is based on f(x) = 3x^2 - 2x^3 over the domain [0,1].
     * @param BrushRadius The brush's radius in [cm]
     * @param EffectiveRadius The effective brush radius in [cm] (region where brush strength = 1)
     * @param Distance The distance from the center of the brush in [cm]
     */
    float CalculateSmoothBrushStrength(float BrushRadius, float EffectiveRadius, float Distance);
};