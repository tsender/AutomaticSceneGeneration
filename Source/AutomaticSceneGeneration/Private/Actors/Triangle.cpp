#include "Actors/Triangle.h"

ATriangle::ATriangle()
{
    PrimaryActorTick.bCanEverTick = true;

    RootComponent = CreateAbstractDefaultSubobject<USceneComponent>(TEXT("Root Component"));
    TerrainMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Terrain Mesh"));
    TerrainMesh->AttachTo(RootComponent);
}

void ATriangle::BeginPlay()
{
    Super::BeginPlay();

    IdxA = 0;
    IdxB = 1;
    IdxC = 2;
    GenerateMesh();
}

void ATriangle::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ATriangle::GenerateMesh()
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
        for (int i = 0; i< Recursions; i++)
        {
            //  Subdivide each face
            for (int j = 0; j < Triangles.Num()/3; j++)
            {
                Subdivide(Triangles[IdxA], Triangles[IdxB], Triangles[IdxC]);
            }

            // Clear the vertex list to remove old vertices
            Vertices.Empty();
            Vertices = NewVertices;

            Triangles.Empty();
            NewTriangles = Triangles;
            NewTriangles.Empty();

            IdxA = 0;
            IdxB = 1;
            IdxC = 2;

            VertexDict.Empty();
            IndicesDict.Empty();
        }
        
        TerrainMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);
    }
}

void ATriangle::Subdivide(int a, int b, int c)
{

}