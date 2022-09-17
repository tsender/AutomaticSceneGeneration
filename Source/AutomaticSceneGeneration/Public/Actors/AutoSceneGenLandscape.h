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

// Landscape falloffs types
enum ELandscapeFalloff
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

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public: /****************************** AActor Overrides ******************************/
	virtual void Tick(float DeltaTime) override;

public: /****************************** AAutoSceneGenLandscape ******************************/
    /**
     * Create the base (flat) square mesh with the specified parameters
     * @param Location The new landscape origin (which is the lower-left corner of the landscape)
     * @param Size The side-length in [cm] of the landscape along the X and Y dimensions (this is a square landscape)
     * @param NumSubdivisions The number of times the two base triangles should be subdivided. The landscape will have 2^NumSubdivisions triangles along each edge.
     *      Each vertex in the mesh will be spaced Size/(2^NumSubdivisions) [cm] apart in a grid.
     */
    bool CreateBaseMesh(FVector Location, float Size, int Subdivisions);

    FBox GetLandscapeBoundingBox() const;

    /**
     * Return the smallest bounding box in the XY plane surrounding the point P
     * @param P Point of interest (Z value is irrelevant), relative to landscape origin
     */
    FIntRect GetVertexGridBounds(FVector P) const;

    /**
     * Return the smallest bounding box in the XY plane containing all points in the input array
     * @param P Array of points to find bounding box for (Z values are irrelevant), relative to landscape origin
     */
    FIntRect GetVertexGridBounds(TArray<FVector> Points) const;

    /**
     * Return the smallest bounding box in the XY plane containing points within radius R of P
     * @param P Center point (Z value is irrelevant), relative to landscape origin
     * @param Radius Radius of circle in [cm]
     */
    FIntRect GetVertexGridBoundsWithinRadius(FVector P, float Radius) const;

    /**
     * Finds the smallest bounding box in the XY plane for the rectangle defined by its middle axis and width.
     * The middle axis is defined by the segment AB, which runs down the middle of the rectangle from bottom (A) to top (B).
     * Effectively, if we move the segment AB perpendicularly to the left and right by a distance of Width/2, we will have defined this rectangle.
     * @param A First segment endpoint (bottom point), relative to landscape origin
     * @param B Second segmnet endpoint (top point), relative to landscape origin
     * @param Width Width of the rectangle in [cm]
     * @param bIncludeEndCaps If true, then the bounding box will also contain all vertices within a radius of Width/2 from A and B in the XY plane creating a planar capsule of vertices
     */
    FIntRect GetVertexGridBoundsWithinRectangle(FVector A, FVector B, float Width, bool bIncludeEndCaps) const;

    /**
     * Finds the vertex buffer indices that are within the radius of the specified point
     * @param P Center point to search from, relative to landscape origin
     * @param Radius Search radius in [cm]
     * @param Indices Array to write vertex buffer indices to
     */
    void GetVertexIndicesWithinRadius(FVector P, float Radius, TArray<int32> &Indices) const;

    /**
     * Finds the vertex grid coordinates that are within the radius of the specified point
     * @param P Center point to search from, relative to landscape origin
     * @param Radius Search radius in [cm]
     * @param VertexGridCoords Array to write XY coordinates to
     */
    void GetVertexGridCoordinatesWithinRadius(FVector P, float Radius, TArray<FIntPoint> &VertexGridCoords) const;

    /**
     * Finds the four vertex buffer indices that surround the specified point
     * @param P Point to find the surrounding vertex buffer indices for, relative to landscape origin
     * @param Indices Array to write vertex buffer indices to
     */
    void GetSurroundingVertexIndices(FVector P, TArray<int32> &Indices) const;

    /**
     * Finds the four vertex grid coordinates that surround the specified point
     * @param P Point to find the surrounding vertex grid coordinates for, relative to landscape origin
     * @param VertexGridCoords Array to write XY coordinates to
     */
    void GetSurroundingVertexGridCoordinates(FVector P, TArray<FIntPoint> &VertexGridCoords) const;

    /**
     * Finds all vertiex buffer indices for vertices that lie within the rectangle defined by its middle axis and width.
     * The middle axis is defined by the segment AB, which runs down the middle of the rectangle from bottom (A) to top (B).
     * Effectively, if we move the segment AB perpendicularly to the left and right by a distance of Width/2, we will have defined this rectangle.
     * @param A First segment endpoint (bottom point), relative to landscape origin
     * @param B Second segmnet endpoint (top point), relative to landscape origin
     * @param Width Width of the rectangle in [cm]
     * @param bIncludeEndCaps If true, then Indices will also contain all vertices within a radius of Width/2 from A and B in the XY plane creating a planar capsule of vertices
     * @param Indices Array to write vertex buffer indices to
     */
    void GetVertexIndicesWithinRectangle(FVector A, FVector B, float Width, bool bIncludeEndCaps, TArray<int32> &Indices) const;

    /**
     * Return the landscape's elevation (relative to the landscape origin) at the specified point using line-tracing. 
     * If line-tracing is unsuccessful (unlikely), then it will return the average height of the vertex's four surrounding vertices.
     * @param P Point to find landscape's Z coordinate at
     */
    float GetLandscapeElevation(FVector P) const;

    /**
     * Change the Z height of the landscape within the specified circular region
     * @param Center Center of desired region, relative to landscape origin
     * @param Radius Radius of region to be modified in [cm]
     * @param DeltaZ The maximum amount of change in the Z direction in [cm]
     * @param bRelative If true, then landscape Z height is modified relative to the original value. Otherwise, DeltaZ indicates the Z position to use (at full brush strength) relative to the landscape origin.
     * @param RadialStrength Indicates the distance from the center point (as a fraction from 0 to 1) at which the brush looses strength and falloff begins. 0 = falloff begins immediately, 1 = no falloff.
     * @param FalloffType Type of falloff to use
     */
    void LandscapeEditSculptCircularPatch(FVector Center, float Radius, float DeltaZ, bool bRelative, float RadialStrength, uint8 FalloffType);

    /**
     * Flatten the landscape within the specified circular region to the height at the center point. Uses a line trace to determine the Z location of the landscape at the specified point.
     * If line trace is unsuccessful, will use the average Z height of the four surrounding vertices.
     * @param Center Center point of desired flat region, relative to landscape origin. Used to determine the height to flatten to
     * @param Radius Radius of region to be modified in [cm]
     * @param RadialStrength Indicates the distance from the center point (as a fraction from 0 to 1) at which the brush looses strength and falloff begins. 0 = falloff begins immediately, 1 = no falloff.
     * @param FalloffType Type of falloff to use
     */
    void LandscapeEditFlattenCircularPatch(FVector Center, float Radius, float RadialStrength, uint8 FalloffType);

    /**
     * Add or subtract a ramp (prism-shpaped object) from A to B. The region affected is based on the rectangle defined by the middle axis AB and the width (see GetVertexIndicesWithinRectangle()).
     * If the height of A and B are different, then the end result is actually a ramp (one end higher than the other) with the specified width.
     * If the height of A and B are the same, then the end result is a raised/lowered piece of land over the specified rectangle.
     * @param A First segment endpoint of the rectangle's middle axis (bottom point), relative to landscape origin
     * @param B Second segmnet endpoint of the rectangle's middle axis (top point), relative to landscape origin
     * @param Width Width of the rectangle in [cm]
     * @param bIncludeEndCaps If true, then Indices will also contain all vertices within a radius of Width/2 from A and B in the XY plane creating a planar capsule of vertices
     * @param bRelative If false, then the landscape Z height of the ramp at any point is equal to the interpolated between A and B. If true, then the Z height is modified relative to that interpolated value.
     * @param RadialStrength Indicates the perpendicular distance from the segment AB (as a fraction from 0 to 1) at which the brush looses strength and falloff begins. 0 = falloff begins immediately, 1 = no falloff.
     * @param FalloffType Type of falloff to use
     */
    void LandscapeEditSculptRamp(FVector A, FVector B, float Width, bool bIncludeEndCaps, bool bRelative, float RadialStrength, uint8 FalloffType);

    /**
     * Recomputes normal vectors and updates the actual landscape mesh. Should be called after editing/sculpting the landscape.
     */
    void PostSculptUpdate();

    /**
     * Reset the landscape back to the base mesh (i.e. a flat plane with many vertices)
     */
    void ResetToBaseMesh();

    /**
     * Set the material for the landscpae
     * @param NewMaterial The new landscape material
     */
    void SetMaterial(UMaterialInterface* NewMaterial);

private: /****************************** AAutoSceneGenLandscape ******************************/    
    UPROPERTY(EditAnywhere)
	class UAnnotationComponent* AnnotationComponent;
    
    UPROPERTY(EditAnywhere)
    class UProceduralMeshComponent* LandscapeMesh;

    TArray<FVector> Vertices;

    TArray<int32> Triangles;

    TArray<FVector> Normals;

    TArray<FVector2D> UV0;

    TArray<FLinearColor> VertexColors;

    TArray<FProcMeshTangent> Tangents;

    bool bCreatedBaseMesh = false;

    // The side-length in [cm] of the landscape along the X and Y dimensions
    float LandscapeSize = 100.;

    // Horizontal distance in [cm] between adjacent vertices in the landscape mesh
    float VertexSeparation;
    
    // Position of lower left corner in XY plane of the landscape (Z coordinate is ignored)
    FVector LowerLeftCorner;

    // Bounding box for entire landscape
    FBox BoundingBox;

    // The number of times the two base triangles should be subdivided. The landscape will have 2^NumSubdivisions triangles along each edge.
    // Each vertex in the mesh will be spaced Size/(2^NumSubdivisions) [cm] apart in a grid.
    int NumSubdivisions = 0;

    // The vertex grid map is a XY grid that maps a vertex's (x,y) integer coordinate to a normalized grid with 1 unit spacing.
    // The lower left is (0,0) and the upper right is (N,N), following Unreal's left-hand coordinate system (X points up, Y points right).
    // This allows us to more quickly lookup the index in the Vertices buffer that corresponds to a certain coordinate.
    TArray<TArray<int32> > VertexGridMap;

    // Clear all vertex-related arrays
    void ClearVertexArrays();

    // Update the Z bounds of the landscape bounding box
    void UpdateZBounds(float NewZ);

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

    /**
     * Computes the normals for the specified vertex (based on grid index)
     * @param V Vertex grid index of desired point
     */
    FVector GetVertexNormal(FIntPoint V);

    /**
     * Recalculates normals for the appropriate vertices based on the region that was modiifed
     * @param Bounds Indicates the smallest bounding box of vertex grid indices containing all vertices that were modified 
     */
    void UpdateNormals(FIntRect Bounds);
};