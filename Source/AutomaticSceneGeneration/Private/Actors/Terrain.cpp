#include "Actors/Terrain.h"
#include "KismetProceduralMeshLibrary.h"

ATerrain::ATerrain()
{
    PrimaryActorTick.bCanEverTick = true;

    RootComponent = CreateAbstractDefaultSubobject<USceneComponent>(TEXT("Root Component"));
    TerrainMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Terrain Mesh"));
    TerrainMesh->AttachTo(RootComponent);
}

void ATerrain::BeginPlay()
{
    Super::BeginPlay();

    IdxA = 0;
    IdxB = 1;
    IdxC = 2;
    GenerateMesh();
}

void ATerrain::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ATerrain::GenerateMesh()
{
    // Vertices.Emplace(FVector(0,0,0));
    // Vertices.Emplace(FVector(0,500,0));
    // Vertices.Emplace(FVector(500,0,0));
    Vertices = {FVector(0,0,0), FVector(0,500,0), FVector(500,0,0)};

    // Triangles.Emplace(0);
    // Triangles.Emplace(1);
    // Triangles.Emplace(2);
    Triangles = {0,1,2};

    TerrainMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);

    // This is the blank list we be writing new vertex arrangement to. We have to make sure that to begin with, it is the smae as the vertices array
    NewVertices = Vertices;

    if (Recursions > 0)
    {
        for (int i = 0; i < Recursions; i++)
        {
            //  Subdivide each face
            for (int j = 0; j < Triangles.Num()/3; j++)
            {
                Subdivide(Triangles[IdxA], Triangles[IdxB], Triangles[IdxC]);
            }

            Vertices.Empty();
            Vertices = NewVertices;

            Triangles.Empty();
            Triangles = NewTriangles;
            NewTriangles.Empty();

            IdxA = 0;
            IdxB = 1;
            IdxC = 2;

            VertexDict.Empty();
            IndicesDict.Empty();
        }

        TerrainMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);
    }

    // Calculate and update normals
    TArray<FVector2D> EmptyUVs;
    TArray<FLinearColor> EmptyFLinearColor;
    TArray<FProcMeshTangent> EmptyTangents;
    UKismetProceduralMeshLibrary::CalculateTangentsForMesh(Vertices, Triangles, EmptyUVs, Normals, Tangents);
    TerrainMesh->UpdateMeshSection_LinearColor(0, Vertices, Normals, EmptyUVs, EmptyFLinearColor, EmptyTangents);
}

void ATerrain::Subdivide(int a, int b, int c)
{
    // Get endpoints
    FVector va = Vertices[a];
    FVector vb = Vertices[b];
    FVector vc = Vertices[c];

    // Get midpoints
    FVector vab = FMath::Lerp(va, vb, 0.5);
    FVector vbc = FMath::Lerp(vb, vc, 0.5);
    FVector vca = FMath::Lerp(vc, va, 0.5);

    // Set endpoint idxs
    i_a = a;
    i_b = b;
    i_c = c;

    bool b_vab_duplicate = false;
    bool b_vbc_duplicate = false;
    bool b_vca_duplicate = false;

    // Check for midpoint duplicates
    for (int i=0; i < VertexDict.Num(); i++)
    {
        if (vab == VertexDict[i])
        {
            b_vab_duplicate = true;
            i_ab = IndicesDict[i];
        }
        if (vbc == VertexDict[i])
        {
            b_vbc_duplicate = true;
            i_bc = IndicesDict[i];
        }
        if (vca == VertexDict[i])
        {
            b_vca_duplicate = true;
            i_ca = IndicesDict[i];
        }
    }

    // If no duplicates found
    if (!b_vab_duplicate)
    {
        NewVertices.Add(vab);
        VertexDict.Add(vab);
        IndicesDict.Add(NewVertices.Num()-1);
        i_ab = NewVertices.Num()-1;
    }
    if (!b_vbc_duplicate)
    {
        NewVertices.Add(vbc);
        VertexDict.Add(vbc);
        IndicesDict.Add(NewVertices.Num()-1);
        i_bc = NewVertices.Num()-1;
    }
    if (!b_vca_duplicate)
    {
        NewVertices.Add(vca);
        VertexDict.Add(vca);
        IndicesDict.Add(NewVertices.Num()-1);
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

    IdxA += 3;
    IdxB += 3;
    IdxC += 3;
}

void ATerrain::AlterTerrain(FVector ImpactPoint)
{
    for (int i=0; i < Vertices.Num(); i++)
    {
        if (FVector(Vertices[i] - ImpactPoint).Size() < 800.)
        {
            Vertices[i] = Vertices[i] - FVector(0,0,150);
            TerrainMesh->UpdateMeshSection(0, Vertices, Normals, UV0, UpVertexColors, Tangents);
        }
    }
}