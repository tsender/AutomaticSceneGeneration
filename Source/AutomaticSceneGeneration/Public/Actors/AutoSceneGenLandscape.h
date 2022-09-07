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

    /**
     * Return the smallest bounding box of vertex grid indices that contain a circle of radius R centered at P
     * @param P Center point (Z value is irrelevant)
     * @param Radius Radius of circle
     */
    FIntRect GetVertexGridBoundsWithinRadius(FVector P, float Radius);

    /**
     * Return the smallest bounding box of grid indices surrounding the point P
     * @param P Point of interest (Z value is irrelevant)
     */
    FIntRect GetVertexGridBounds(FVector P);

    /**
     * Return the smallest bounding box of grid indices surrounding all points in the input array
     * @param P Array of points to find bounding box for (Z values are irrelevant)
     */
    FIntRect GetVertexGridBounds(TArray<FVector> Points);

    /**
     * Finds the vertex buffer indices that are within the radius of the specified point
     * @param P Center point to search from (does not have to be coicident with a vertex), only the XY coordinates will be used
     * @param Radius Search radius
     * @param Indices Array to write vertex buffer indices to
     */
    void GetVertexIndicesWithinRadius(FVector P, float Radius, TArray<int32> &Indices);

    /**
     * Finds the vertex grid coordinates that are within the radius of the specified point
     * @param P Center point to search from (does not have to be coicident with a vertex), only the XY coordinates will be used
     * @param Radius Search radius
     * @param VertexGridCoords Array to write XY coordinates to
     */
    void GetVertexGridCoordinatesWithinRadius(FVector P, float Radius, TArray<FIntPoint> &VertexGridCoords);

    /**
     * Finds the four vertex buffer indices that surround the specified point
     * @param P Point to find the surrounding vertex buffer indices for
     * @param Indices Array to write vertex buffer indices to
     */
    void GetSurroundingVertexIndices(FVector P, TArray<int32> &Indices);

    /**
     * Finds the four vertex grid coordinates that surround the specified point
     * @param P Point to find the surrounding vertex grid coordinates for
     * @param VertexGridCoords Array to write XY coordinates to
     */
    void GetSurroundingVertexGridCoordinates(FVector P, TArray<FIntPoint> &VertexGridCoords);

    /**
     * Finds all vertiex buffer indices for vertices that are at most MaxPerpDistance away (in the XY plane) from the segment A to B.
     * @param A First segment endpoint
     * @param B Second segmnet endpoint
     * @param Indices Array to write vertex buffer indices to
     * @param bIncludeEndCaps If true, then Indices will also contain all vertices within an a distance of MaxPerpDistance from A and B in the XY plane
     */
    void GetVertexIndicesWithinDistanceToLine(FVector A, FVector B, float MaxPerpDistance, TArray<int32> &Indices, bool bIncludeEndCaps = false);

    /**
     * Return the landscape's height at the specified point using line-tracing. If line-tracing is unsuccessful (unlikely), then it will return the
     * average height of the vertex's four surrounding vertices.
     */
    float GetLandscapeHeight(FVector P);

    /**
     * Add a 2D bell-shaped curve to the existing landscape (can be up- or downward facing)
     * @param Mean The center of the gaussian (only the XY coordinates are used)
     * @param Stddev The standard deviation in [cm], applies to all directions (we will only modify vertices within 5 stddev of the center)
     * @param DeltaZ The maximum amount of change in the Z direction in [cm]
     */
    void LandscapeEditSculptGaussian(FVector Mean, float Stddev, float DeltaZ);

    /**
     * Change the Z height of the landscape within the specified circular region
     * @param Center Center of desired region
     * @param Radius Radius of region to be modified
     * @param DeltaZ The maximum amount of change in the Z direction in [cm]
     * @param bRelative If true, then landscape Z height is modified relative to the original value. Otherwise, DeltaZ indicates the actual Z position to use (at full brush strength).
     * @param BrushFalloff Indicates the distance from the center point (as a fraction from 0 to 1) at which the brush looses strength and falloff begins
     * @param FalloffType Type of falloff to use
     */
    void LandscapeEditSculptCircularPatch(FVector Center, float Radius, float DeltaZ, bool bRelative, float BrushFalloff, uint8 FalloffType);

    /**
     * Flatten the landscape within the specified circular region. Uses a line trace to determine the Z location of the landscape at the specified point.
     * If line trace is unsuccessful, will use the average Z height of the four surrounding vertices.
     * @param Center Center of desired region
     * @param Radius Radius of region to be modified
     * @param BrushFalloff Indicates the distance from the center point (as a fraction from 0 to 1) at which the brush looses strength and falloff begins
     * @param FalloffType Type of falloff to use
     */
    void LandscapeEditFlattenCircularPatch(FVector Center, float Radius, float BrushFalloff, uint8 FalloffType);

    // void LandscapeEditSculptRamp(FVector );

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
     * Falloff function is based on f(x) = 0.5cos(pi(x-1)) + 0.5 over the domain [0,1].
     * @param BrushRadius The brush's radius in [cm]
     * @param EffectiveRadius The effective brush radius in [cm] (region where brush strength = 1)
     * @param Distance The distance from the center of the brush in [cm]
     */
    float CalculateSmoothBrushStrength(float BrushRadius, float EffectiveRadius, float Distance);

    /**
     * Compute brush strength based on specified falloff. Max strength = 1 and occurs when Distance <= EffectiveRadius. Min strength = 0 and occurs when Distance >= BrushRadius.
     * @param BrushRadius The brush's radius in [cm]
     * @param EffectiveRadius The effective brush radius in [cm] (region where brush strength = 1)
     * @param Distance The distance from the center of the brush in [cm]
     */
    float CalculateBrushStrength(float BrushRadius, float EffectiveRadius, float Distance, uint8 FalloffType);
};