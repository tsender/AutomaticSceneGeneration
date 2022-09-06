#include "Actors/AutoSceneGenLandscape.h"
#include "KismetProceduralMeshLibrary.h"
#include "AutoSceneGenLogging.h"

AAutoSceneGenLandscape::AAutoSceneGenLandscape()
{
    PrimaryActorTick.bCanEverTick = true;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root Component"));
    TerrainMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Terrain Mesh"));
    TerrainMesh->SetupAttachment(RootComponent);
}

void AAutoSceneGenLandscape::BeginPlay()
{
    Super::BeginPlay();
}

void AAutoSceneGenLandscape::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

bool AAutoSceneGenLandscape::CreateBaseMesh(FVector Location, float Size, int Subdivisions)
{
    SetActorLocation(Location);
    SetActorRotation(FRotator(0));
    LandscapeSize = Size;
    NumSubdivisions = Subdivisions;
    VertexSeparation = Size/FMath::Pow(2, NumSubdivisions);

    // FVector Offset = FVector(Size/2., Size/2., 0.) - Location;
    FVector Offset = FVector(0.);
    Vertices.Emplace(FVector(0.,0.,0.) - Offset);       // 0 - Lower left
    Vertices.Emplace(FVector(Size,0.,0.) - Offset);     // 1 - Upper left
    Vertices.Emplace(FVector(0.,Size,0.) - Offset);    // 2 - Lower right
    Vertices.Emplace(FVector(Size,Size,0.) - Offset);   // 3 - Upper right

    LowerLeftCorner = Vertices[0];
    
    // Lower left base triangle
    Triangles.Emplace(0);
    Triangles.Emplace(1);
    Triangles.Emplace(2);

    // Upper right base triangle
    Triangles.Emplace(3);
    Triangles.Emplace(2);
    Triangles.Emplace(1);

    SubdivideMesh(NumSubdivisions, Vertices, Triangles);

    int32 NumSideVertices = FMath::Pow(2, NumSubdivisions) + 1; // Number of vertices per landscape edge
    if (Vertices.Num() != NumSideVertices * NumSideVertices)
    {
        UE_LOG(LogASG, Error, TEXT("Created landscaoe has %i vertice sinstead of %i"), Vertices.Num(),  NumSideVertices * NumSideVertices);
        return false;
    }

    TArray<int32> ZeroArray;
    ZeroArray.Init(NumSideVertices, 0);
    for (int32 i = 0; i < NumSideVertices; i++)
        VertexGridMap.Emplace(ZeroArray);

    // VertexGridMap: imagine a grid with Y- pointing right and X- pointing up, with vertices spaced 1 unit apart
    // The (x,y) element is the mesh vertex index for the corresponding unnormalized (x,y) location
    for (int32 i = 0; i < NumSideVertices; i++)
    {
        for (int32 j = 0; j < NumSideVertices; j++)
        {
            int32 idx = i*NumSideVertices + j;
            FVector v = Vertices[idx] - LowerLeftCorner;

            int32 x = FMath::RoundToInt(FMath::Abs(v.X/VertexSeparation));
            int32 y = FMath::RoundToInt(FMath::Abs(v.Y/VertexSeparation));
            VertexGridMap[x][y] = idx;
        }
    }

    TerrainMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);

    // Calculate and update normals
    // TArray<FVector2D> EmptyUVs;
    // TArray<FLinearColor> EmptyFLinearColor;
    // TArray<FProcMeshTangent> EmptyTangents;
    // UKismetProceduralMeshLibrary::CalculateTangentsForMesh(Vertices, Triangles, EmptyUVs, Normals, Tangents);
    // TerrainMesh->UpdateMeshSection_LinearColor(0, Vertices, Normals, EmptyUVs, EmptyFLinearColor, EmptyTangents);

    return true;
}

void AAutoSceneGenLandscape::SubdivideTriangle(int32 i_a, int32 i_b, int32 i_c, const TArray<FVector> &MeshVertices, TArray<FVector> &AddedVertices, TArray<int32> &AddedIndices, TArray<FVector> &NewVertices, TArray<int32> &NewTriangles)
{
    // Get endpoints
    FVector va = MeshVertices[i_a];
    FVector vb = MeshVertices[i_b];
    FVector vc = MeshVertices[i_c];

    // Get midpoints
    FVector vab = FMath::Lerp(va, vb, 0.5);
    FVector vbc = FMath::Lerp(vb, vc, 0.5);
    FVector vca = FMath::Lerp(vc, va, 0.5);

    // Idxs for midpoints
    int32 i_ab, i_bc, i_ca;

    bool b_vab_duplicate = false;
    bool b_vbc_duplicate = false;
    bool b_vca_duplicate = false;

    // Check for midpoint duplicates
    for (int32 i=0; i < AddedVertices.Num(); i++)
    {
        if (vab == AddedVertices[i])
        {
            b_vab_duplicate = true;
            i_ab = AddedIndices[i];
        }
        if (vbc == AddedVertices[i])
        {
            b_vbc_duplicate = true;
            i_bc = AddedIndices[i];
        }
        if (vca == AddedVertices[i])
        {
            b_vca_duplicate = true;
            i_ca = AddedIndices[i];
        }
    }

    // If no midpoint duplicates found
    if (!b_vab_duplicate)
    {
        NewVertices.Add(vab);
        AddedVertices.Add(vab);
        AddedIndices.Add(NewVertices.Num()-1);
        i_ab = NewVertices.Num()-1;
    }
    if (!b_vbc_duplicate)
    {
        NewVertices.Add(vbc);
        AddedVertices.Add(vbc);
        AddedIndices.Add(NewVertices.Num()-1);
        i_bc = NewVertices.Num()-1;
    }
    if (!b_vca_duplicate)
    {
        NewVertices.Add(vca);
        AddedVertices.Add(vca);
        AddedIndices.Add(NewVertices.Num()-1);
        i_ca = NewVertices.Num()-1;
    }

    // 1st triangle (lower left)
    NewTriangles.Emplace(i_a);
    NewTriangles.Emplace(i_ab);
    NewTriangles.Emplace(i_ca);

    // 2nd triangle (lower right)
    NewTriangles.Emplace(i_ca);
    NewTriangles.Emplace(i_bc);
    NewTriangles.Emplace(i_c);

    // 3rd triangle (upper left)
    NewTriangles.Emplace(i_ab);
    NewTriangles.Emplace(i_b);
    NewTriangles.Emplace(i_bc);

    // 4th triangle (middle triangle)
    NewTriangles.Emplace(i_ab);
    NewTriangles.Emplace(i_bc);
    NewTriangles.Emplace(i_ca);
}

void AAutoSceneGenLandscape::SubdivideMesh(int32 Subdivisions, TArray<FVector> &MeshVertices, TArray<int32> &MeshTriangles)
{
    if (Subdivisions > 0)
    {
        // Begin creating the new list of mesh vertices
        TArray<FVector> NewVertices = MeshVertices;

        for (int32 i = 0; i < Subdivisions; i++)
        {
            // First idx in triangle
            int32 IdxA = 0;

            // Keep a record of all new vertices (used by SubdivideTriangle())
            TArray<FVector> AddedVertices;

            // Keeps a record of indices for all new vertices (used by SubdivideTriangle())
            TArray<int32> AddedIndices;

            // The new set of triangles to render (triangles added in SubdivideTriangle())
            TArray<int32> NewTriangles;
            
            //  Subdivide each face
            for (int32 j = 0; j < MeshTriangles.Num()/3; j++)
            {
                SubdivideTriangle(MeshTriangles[IdxA], MeshTriangles[IdxA+1], MeshTriangles[IdxA+2], MeshVertices, AddedVertices, AddedIndices, NewVertices, NewTriangles);
                IdxA += 3;
            }

            // Update mesh data
            MeshVertices = NewVertices;
            MeshTriangles = NewTriangles;
        }
    }
}

float AAutoSceneGenLandscape::CalculateLinearBrushStrength(float BrushRadius, float EffectiveRadius, float Distance)
{
    if (BrushRadius <= 0) return 0.;
    // if (Distance <= EffectiveRadius) return 1.;
    // if (Distance >= BrushRadius) return 0.;
    return FMath::Clamp<float>(1. - (Distance - EffectiveRadius) / (BrushRadius - EffectiveRadius), 0., 1.);
}

float AAutoSceneGenLandscape::CalculateSmoothBrushStrength(float BrushRadius, float EffectiveRadius, float Distance)
{
    float x = CalculateLinearBrushStrength(BrushRadius, EffectiveRadius, Distance);
    return x*x*(3. - 2.*x);
}

// void AAutoSceneGenLandscape::AlterTerrain(FVector ImpactPoint)
// {
//     for (int i=0; i < Vertices.Num(); i++)
//     {
//         if (FVector(Vertices[i] - ImpactPoint).Size() < 800.)
//         {
//             Vertices[i] = Vertices[i] - FVector(0,0,150);
//             TerrainMesh->UpdateMeshSection(0, Vertices, Normals, UV0, UpVertexColors, Tangents);
//         }
//     }
// }

void AAutoSceneGenLandscape::GetVertexIndicesWithinRadius(FVector P, float Radius, TArray<int32> &Indices)
{    
    // Compute range of (x,y) candidate vertices for the VertexGridMap
    FVector v = P - LowerLeftCorner;
    int32 xmin = FMath::CeilToInt((v.X - Radius)/VertexSeparation);
    int32 xmax = FMath::FloorToInt((v.X + Radius)/VertexSeparation);
    int32 ymin = FMath::CeilToInt((v.Y - Radius)/VertexSeparation);
    int32 ymax = FMath::FloorToInt((v.Y + Radius)/VertexSeparation);

    xmin = FMath::Clamp<int32>(xmin, 0, FMath::Pow(2, NumSubdivisions));
    xmax = FMath::Clamp<int32>(xmax, 0, FMath::Pow(2, NumSubdivisions));
    ymin = FMath::Clamp<int32>(ymin, 0, FMath::Pow(2, NumSubdivisions));
    ymax = FMath::Clamp<int32>(ymax, 0, FMath::Pow(2, NumSubdivisions));

    // Search through candidate vertices
    P.Z = 0;
    for (int32 i = xmin; i <= xmax; i++)
    {
        for (int32 j = ymin; j <= ymax; j++)
        {
            FVector Vertex = Vertices[VertexGridMap[i][j]];
            Vertex.Z = 0;
            if ((Vertex - P).Size() <= Radius)
                Indices.Emplace(VertexGridMap[i][j]);
        }
    }
}

void AAutoSceneGenLandscape::GetVertexMapCoordinatesWithinRadius(FVector P, float Radius, TArray<FIntPoint> &VertexMapCoords)
{
    // Compute range of (x,y) candidate vertices for the VertexGridMap
    FVector v = P - LowerLeftCorner;
    int32 xmin = FMath::CeilToInt((v.X - Radius)/VertexSeparation);
    int32 xmax = FMath::FloorToInt((v.X + Radius)/VertexSeparation);
    int32 ymin = FMath::CeilToInt((v.Y - Radius)/VertexSeparation);
    int32 ymax = FMath::FloorToInt((v.Y + Radius)/VertexSeparation);

    xmin = FMath::Clamp<int32>(xmin, 0, FMath::Pow(2, NumSubdivisions));
    xmax = FMath::Clamp<int32>(xmax, 0, FMath::Pow(2, NumSubdivisions));
    ymin = FMath::Clamp<int32>(ymin, 0, FMath::Pow(2, NumSubdivisions));
    ymax = FMath::Clamp<int32>(ymax, 0, FMath::Pow(2, NumSubdivisions));

    // Search through candidate vertices
    P.Z = 0;
    for (int32 i = xmin; i <= xmax; i++)
    {
        for (int32 j = ymin; j <= ymax; j++)
        {
            FVector Vertex = Vertices[VertexGridMap[i][j]];
            Vertex.Z = 0;
            if ((Vertex - P).Size() <= Radius)
                VertexMapCoords.Emplace(FIntPoint(i,j));
        }
    }
}

void AAutoSceneGenLandscape::LandscapeEditSculptGaussian(FVector Mean, float Stddev, float Height)
{
    TArray<int32> AffectedVertices;
    GetVertexIndicesWithinRadius(Mean, 5*Stddev, AffectedVertices);

    Mean.Z =  0;
    for (int32 idx : AffectedVertices)
    {
        FVector v = Vertices[idx];
        v.Z = 0;
        Vertices[idx].Z += Height * FMath::Exp(-1/(2*Stddev) * FMath::Pow((Mean - v).Size(), 2));
    }
}