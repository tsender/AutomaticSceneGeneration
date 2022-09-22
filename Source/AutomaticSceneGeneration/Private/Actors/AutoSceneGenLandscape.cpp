#include "Actors/AutoSceneGenLandscape.h"
#include "Components/AnnotationComponent.h"
#include "Kismet/GameplayStatics.h"
#include "AutoSceneGenLogging.h"
#include <chrono>

AAutoSceneGenLandscape::AAutoSceneGenLandscape()
{
    PrimaryActorTick.bCanEverTick = true;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root Component"));
    LandscapeMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Terrain Mesh"));
    LandscapeMesh->SetupAttachment(RootComponent);

    AnnotationComponent = CreateDefaultSubobject<UAnnotationComponent>(TEXT("Annotation Component"));
}

void AAutoSceneGenLandscape::BeginPlay()
{
    Super::BeginPlay();

    bCreatedBaseMesh = false;
    ClearVertexArrays();

    // TODO: Set annotation colors from details panel
    // TODO: Need custom traversability material (flat regions are traversable, high slopes are not, etc.)
    AnnotationComponent->AddAnnotationColor(EAnnotationColor::Traversable, FColor(255, 255, 255, 255)); // White
    AnnotationComponent->AddAnnotationColor(EAnnotationColor::SemanticSegmentation, FColor(0, 255, 0, 255)); // Green
}

void AAutoSceneGenLandscape::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ClearVertexArrays();
}

void AAutoSceneGenLandscape::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

bool AAutoSceneGenLandscape::CreateBaseMesh(FVector Location, float Size, int Subdivisions)
{
    if (bCreatedBaseMesh && GetActorLocation() == Location && LandscapeSize == Size && NumSubdivisions == Subdivisions)
    {
        ResetToBaseMesh();
        return true;
    }
    if (Subdivisions < 0)
    {
        UE_LOG(LogASG, Warning, TEXT("Cannot request landscape with negative subdivisions."));
        bCreatedBaseMesh = false;
        return false;
    }
    
    SetActorLocation(Location);
    SetActorRotation(FRotator(0));
    LandscapeSize = Size;
    NumSubdivisions = Subdivisions;
    VertexSeparation = Size/FMath::Pow(2, NumSubdivisions);

    ClearVertexArrays();

    LowerLeftCorner = Location;
    BoundingBox.Min = FVector(0.);
    BoundingBox.Max = FVector(LandscapeSize, LandscapeSize, 0.);

    int32 MaxIdx = FMath::Pow(2, NumSubdivisions);
    int32 NumSideVertices = MaxIdx + 1; // Number of vertices per landscape edge

    TArray<int32> ZeroArray;
    ZeroArray.Init(0, NumSideVertices);
    VertexGridMap.Init(ZeroArray, NumSideVertices);

    // Create vertices array and fill in vertex grid map
    // VertexGridMap: imagine a grid with Y- pointing right and X- pointing up, with vertices spaced 1 unit apart
    // The (x,y) element is the mesh vertex index for the corresponding unnormalized (x,y) location
    for (int32 i = 0; i < NumSideVertices; i++)
    {
        for (int32 j = 0; j < NumSideVertices; j++)
        {
            Vertices.Emplace(i*FVector(VertexSeparation, 0., 0.) + j*FVector(0., VertexSeparation, 0.)); // Position is relative to root component
            UV0.Emplace(FVector2D(j/(NumSideVertices-1.), i/(NumSideVertices-1.)));
            VertexGridMap[i][j] = i*NumSideVertices + j;
        }
    }

    // Create triangles
    for (int32 i = 0; i < NumSideVertices-1; i++)
    {
        for (int32 j = 0; j < NumSideVertices-1; j++)
        {
            // Triangle vertices should be added in CCW order to ensure the texture faces towards us (upwards in our case)
            // Lower triangle
            Triangles.Emplace(VertexGridMap[i][j]);
            Triangles.Emplace(VertexGridMap[i][j+1]);
            Triangles.Emplace(VertexGridMap[i+1][j]);

            // Upper triangle
            Triangles.Emplace(VertexGridMap[i+1][j+1]);
            Triangles.Emplace(VertexGridMap[i+1][j]);
            Triangles.Emplace(VertexGridMap[i][j+1]);
        }
    }

    Normals.Init(FVector(0., 0., 1.), Vertices.Num());

    LandscapeMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, TArray<FLinearColor>(), TArray<FProcMeshTangent>(), true);
    UE_LOG(LogASG, Display, TEXT("Created base landscape mesh of size %f x %f [cm] with %i x %i Vertices"), LandscapeSize, LandscapeSize, NumSideVertices, NumSideVertices);
    UE_LOG(LogASG, Warning, TEXT("Landscape bounds %s"), *BoundingBox.ToString());
    bCreatedBaseMesh = true;
    return true;
}

FBox AAutoSceneGenLandscape::GetLandscapeBoundingBox() const
{
    return BoundingBox;
}

FIntRect AAutoSceneGenLandscape::GetVertexGridBounds(FVector P) const
{
    int32 xmin = FMath::FloorToInt(P.X/VertexSeparation);
    int32 xmax = FMath::CeilToInt(P.X/VertexSeparation);
    int32 ymin = FMath::FloorToInt(P.Y/VertexSeparation);
    int32 ymax = FMath::CeilToInt(P.Y/VertexSeparation);

    xmin = FMath::Clamp<int32>(xmin, 0, FMath::Pow(2, NumSubdivisions));
    xmax = FMath::Clamp<int32>(xmax, 0, FMath::Pow(2, NumSubdivisions));
    ymin = FMath::Clamp<int32>(ymin, 0, FMath::Pow(2, NumSubdivisions));
    ymax = FMath::Clamp<int32>(ymax, 0, FMath::Pow(2, NumSubdivisions));

    return FIntRect(xmin, ymin, xmax, ymax);
}

FIntRect AAutoSceneGenLandscape::GetVertexGridBounds(TArray<FVector> Points) const
{
    int32 xmin = FMath::Pow(2, NumSubdivisions), ymin = FMath::Pow(2, NumSubdivisions);
    int32 xmax = 0, ymax = 0;
    for (FVector P : Points)
    {
        FIntRect Bounds = GetVertexGridBounds(P);
        xmin = FMath::Min<int32>(Bounds.Min.X, xmin);
        ymin = FMath::Min<int32>(Bounds.Min.Y, ymin);
        xmax = FMath::Max<int32>(Bounds.Max.X, xmax);
        ymax = FMath::Max<int32>(Bounds.Max.Y, ymax);
    }
    return FIntRect(xmin, ymin, xmax, ymax);
}

FIntRect AAutoSceneGenLandscape::GetVertexGridBoundsWithinRadius(FVector P, float Radius) const
{
    int32 xmin = FMath::CeilToInt((P.X - Radius)/VertexSeparation);
    int32 xmax = FMath::FloorToInt((P.X + Radius)/VertexSeparation);
    int32 ymin = FMath::CeilToInt((P.Y - Radius)/VertexSeparation);
    int32 ymax = FMath::FloorToInt((P.Y + Radius)/VertexSeparation);

    xmin = FMath::Clamp<int32>(xmin, 0, FMath::Pow(2, NumSubdivisions));
    xmax = FMath::Clamp<int32>(xmax, 0, FMath::Pow(2, NumSubdivisions));
    ymin = FMath::Clamp<int32>(ymin, 0, FMath::Pow(2, NumSubdivisions));
    ymax = FMath::Clamp<int32>(ymax, 0, FMath::Pow(2, NumSubdivisions));

    return FIntRect(xmin, ymin, xmax, ymax);
}

FIntRect AAutoSceneGenLandscape::GetVertexGridBoundsWithinRectangle(FVector A, FVector B, float Width, bool bIncludeEndCaps) const
{
    float HalfWidth = Width/2.;
    FVector ABNorm = (B-A).GetSafeNormal2D();

    // Find vector perpendicular to AB and normalize. ABPerp will be rotated clockwise from AB.
    FVector ABPerp = FVector(0.);
    ABPerp.X = -ABNorm.Y;
    ABPerp.Y = ABNorm.X;

    TArray<FVector> Points;
    Points.Emplace(A + ABPerp * HalfWidth); // Right of A (when looking in direction of AB)
    Points.Emplace(A - ABPerp * HalfWidth); // Left of A (when looking in direction of AB)
    Points.Emplace(B + ABPerp * HalfWidth); // Right of B (when looking in direction of AB)
    Points.Emplace(B - ABPerp * HalfWidth); // Left of B (when looking in direction of AB)

    if (bIncludeEndCaps)
    {
        Points.Emplace(A + ABPerp * HalfWidth - ABNorm * HalfWidth);
        Points.Emplace(A - ABPerp * HalfWidth - ABNorm * HalfWidth);
        Points.Emplace(B + ABPerp * HalfWidth + ABNorm * HalfWidth);
        Points.Emplace(B - ABPerp * HalfWidth + ABNorm * HalfWidth);
    }

    return GetVertexGridBounds(Points);
}

void AAutoSceneGenLandscape::GetVertexIndicesWithinRadius(FVector P, float Radius, TArray<int32> &Indices) const
{    
    FIntRect Bounds = GetVertexGridBoundsWithinRadius(P, Radius);

    // Search through candidate vertices
    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
    {
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
        {
            FVector Vertex = Vertices[VertexGridMap[i][j]];
            if ((Vertex - P).Size2D() <= Radius)
                Indices.Emplace(VertexGridMap[i][j]);
        }
    }
}

void AAutoSceneGenLandscape::GetVertexGridCoordinatesWithinRadius(FVector P, float Radius, TArray<FIntPoint> &VertexGridCoords) const
{
    FIntRect Bounds = GetVertexGridBoundsWithinRadius(P, Radius);

    // Search through candidate vertices
    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
    {
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
        {
            FVector Vertex = Vertices[VertexGridMap[i][j]];
            if ((Vertex - P).Size2D() <= Radius)
                VertexGridCoords.Emplace(FIntPoint(i,j));
        }
    }
}

void AAutoSceneGenLandscape::GetSurroundingVertexIndices(FVector P, TArray<int32> &Indices) const
{
    FIntRect Bounds = GetVertexGridBounds(P);

    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
            Indices.Emplace(VertexGridMap[i][j]);
}

void AAutoSceneGenLandscape::GetSurroundingVertexGridCoordinates(FVector P, TArray<FIntPoint> &VertexGridCoords) const
{
    FIntRect Bounds = GetVertexGridBounds(P);

    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
            VertexGridCoords.Emplace(FIntPoint(i,j));
}

void AAutoSceneGenLandscape::GetVertexIndicesWithinRectangle(FVector A, FVector B, float Width,  bool bIncludeEndCaps, TArray<int32> &Indices) const
{
    float HalfWidth = Width/2.;
    FIntRect Bounds = GetVertexGridBoundsWithinRectangle(A, B, Width, bIncludeEndCaps);

    // Search through candidate vertices
    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
    {
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
        {
            FVector C = Vertices[VertexGridMap[i][j]];
            FVector AC = (C - A).GetSafeNormal2D(); // Norm vector from A to C
            FVector AB = (B - A).GetSafeNormal2D(); // Norm vector from A to B
            FVector BC = (C - B).GetSafeNormal2D(); // Norm vector from B to C

            float AngleBAC = FMath::Acos(AB | AC);
            float AngleABC = FMath::Acos((-AB) | BC);

            if (AngleBAC <= 90. && AngleABC <= 90. && (C-A).Size2D()*FMath::Sin(AngleBAC) <= HalfWidth)
                Indices.Emplace(VertexGridMap[i][j]);

            if (bIncludeEndCaps)
            {
                // Check if in A endcap
                if ((C-A).Size2D() <= HalfWidth)
                    Indices.Emplace(VertexGridMap[i][j]);

                // Check if in B endcap
                if ((C-B).Size2D() <= HalfWidth)
                    Indices.Emplace(VertexGridMap[i][j]);
            }
        }
    }
}

float AAutoSceneGenLandscape::GetLandscapeElevation(FVector P, const TArray<AActor*> &ActorsToIgnore) const
{
    TArray<int32> SurroundingVertices;
    GetSurroundingVertexIndices(P, SurroundingVertices);

    float MinZ = Vertices[SurroundingVertices[0]].Z;
    float MaxZ = MinZ;

    float AvgSurroundingElevation = 0.;
    for (int32 idx : SurroundingVertices)
    {
        if (Vertices[idx].Z < MinZ)
            MinZ = Vertices[idx].Z;

        if (Vertices[idx].Z > MaxZ)
            MaxZ = Vertices[idx].Z;

        AvgSurroundingElevation += Vertices[idx].Z;
    }

    AvgSurroundingElevation /= SurroundingVertices.Num();

    FVector StartTrace = GetActorLocation() + FVector(P.X, P.Y, MaxZ + 1.);
    FVector EndTrace = GetActorLocation() + FVector(P.X, P.Y, MinZ - 1.);
    FHitResult Hit;
    FCollisionQueryParams Params;
	if (ActorsToIgnore.Num())
        Params.AddIgnoredActors(ActorsToIgnore);
	bool bSuccess = GetWorld()->LineTraceSingleByChannel(Hit, StartTrace, EndTrace, ECollisionChannel::ECC_Visibility, Params);

    if (bSuccess)
        return Hit.ImpactPoint.Z - GetActorLocation().Z; // ImpactPoint is in world coordinates, so subtract landscape Z position
    else
        return AvgSurroundingElevation;
}

bool AAutoSceneGenLandscape::GetLandscapeSurfaceData(FVector P, const TArray<AActor*> &ActorsToIgnore, FHitResult &Hit) const
{
    TArray<int32> SurroundingVertices;
    GetSurroundingVertexIndices(P, SurroundingVertices);

    float MinZ = Vertices[SurroundingVertices[0]].Z;
    float MaxZ = MinZ;

    for (int32 idx : SurroundingVertices)
    {
        if (Vertices[idx].Z < MinZ)
            MinZ = Vertices[idx].Z;

        if (Vertices[idx].Z > MaxZ)
            MaxZ = Vertices[idx].Z;

    }

    // For now let's assume zero rotation
    FVector StartTrace = GetActorLocation() + FVector(P.X, P.Y, MaxZ + 1.); // Start above landscape
    FVector EndTrace = GetActorLocation() + FVector(P.X, P.Y, MinZ - 1.); // End below landscape
    FCollisionQueryParams Params;
    if (ActorsToIgnore.Num())
        Params.AddIgnoredActors(ActorsToIgnore);
	return GetWorld()->LineTraceSingleByChannel(Hit, StartTrace, EndTrace, ECollisionChannel::ECC_Visibility, Params);
}

void AAutoSceneGenLandscape::LandscapeEditSculptCircularPatch(FVector Center, float Radius, float DeltaZ, bool bRelative, float RadialStrength, uint8 FalloffType)
{
    if (Radius <= 0.)
    {
        UE_LOG(LogASG, Warning, TEXT("LandscapeEditSculptCircularPatch() - Radius must be positive. Cannot sculpt circular patch."));
        return;
    }

    FIntRect Bounds = GetVertexGridBoundsWithinRadius(Center, Radius);

    // Search through candidate vertices
    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
    {
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
        {
            int32 idx = VertexGridMap[i][j];
            FVector Vertex = Vertices[idx];
            if ((Vertex - Center).Size2D() <= Radius)
            {
                float BrushStrength = CalculateBrushStrength(Radius, FMath::Clamp<float>(RadialStrength, 0., 1.) * Radius, (Center - Vertex).Size2D(), FalloffType);

                if (bRelative)
                    Vertices[idx].Z += FMath::Lerp<float>(0., DeltaZ, BrushStrength);
                else
                    Vertices[idx].Z = FMath::Lerp<float>(Vertices[idx].Z, DeltaZ, BrushStrength);
                UpdateZBounds(Vertices[idx].Z);
            }
        }
    }
    UpdateNormals(Bounds);
}

void AAutoSceneGenLandscape::LandscapeEditFlattenCircularPatch(FVector Center, float Radius, float RadialStrength, uint8 FalloffType)
{
    TArray<AActor*> ActorsToIgnore;
    LandscapeEditSculptCircularPatch(Center, Radius, GetLandscapeElevation(Center, ActorsToIgnore), false, RadialStrength, FalloffType);
}

void AAutoSceneGenLandscape::LandscapeEditSculptRamp(FVector A, FVector B, float Width, bool bIncludeEndCaps, bool bRelative, float RadialStrength, uint8 FalloffType)
{
    if (Width <= 0.)
    {
        UE_LOG(LogASG, Warning, TEXT("LandscapeEditSculptRamp() - Width must be positive. Cannot sculpt ramp."));
        return;
    }
    
    if ((B-A).Size2D() == 0. && bIncludeEndCaps)
    {
        UE_LOG(LogASG, Display, TEXT("LandscapeEditSculptRamp() - A and B are coincident in XY plane, using LandscapeEditSculptCircularPatch() instead"));
        LandscapeEditSculptCircularPatch(A, Width/2., A.Z, false, RadialStrength, FalloffType);
        return;
    }

    float HalfWidth = Width/2.;
    FIntRect Bounds = GetVertexGridBoundsWithinRectangle(A, B, Width, bIncludeEndCaps);

    // Search through candidate vertices
    for (int32 i = Bounds.Min.X; i <= Bounds.Max.X; i++)
    {
        for (int32 j = Bounds.Min.Y; j <= Bounds.Max.Y; j++)
        {
            int32 idx = VertexGridMap[i][j];
            FVector C = Vertices[idx];
            FVector AC = (C - A).GetSafeNormal2D(); // Norm vector from A to C
            FVector AB = (B - A).GetSafeNormal2D(); // Norm vector from A to B
            FVector BC = (C - B).GetSafeNormal2D(); // Norm vector from B to C

            float AngleBAC = FMath::Acos(AB | AC);
            float AngleABC = FMath::Acos((-AB) | BC);

            if (AngleBAC <= PI/2. && AngleABC <= PI/2. && (C-A).Size2D()*FMath::Sin(AngleBAC) <= HalfWidth) // Check if inside rectangle
            {
                float BrushStrength = CalculateBrushStrength(HalfWidth, FMath::Clamp<float>(RadialStrength, 0., 1.) * HalfWidth, (C-A).Size2D()*FMath::Sin(AngleBAC), FalloffType);
                float MaxDeltaZAtC = FMath::Lerp<float>(A.Z, B.Z, (C-A).Size2D()*FMath::Cos(AngleBAC) / (B-A).Size2D());

                if (bRelative)
                    Vertices[idx].Z += FMath::Lerp<float>(0., MaxDeltaZAtC, BrushStrength);
                else
                    Vertices[idx].Z = FMath::Lerp<float>(0., MaxDeltaZAtC, BrushStrength);
                UpdateZBounds(Vertices[idx].Z);
            }
            else if (bIncludeEndCaps)
            {
                if (AngleBAC > PI/2. && (C-A).Size2D() <= HalfWidth) // Check if in A endcap
                {
                    float BrushStrength = CalculateBrushStrength(HalfWidth, FMath::Clamp<float>(RadialStrength, 0., 1.) * HalfWidth, (C-A).Size2D(), FalloffType);
                    
                    if (bRelative)
                        Vertices[idx].Z += FMath::Lerp<float>(0., A.Z, BrushStrength);
                    else
                        Vertices[idx].Z = FMath::Lerp<float>(0., A.Z, BrushStrength);
                    UpdateZBounds(Vertices[idx].Z);
                }
                else if (AngleABC > PI/2. && (C-B).Size2D() <= HalfWidth) // Check if in B endcap
                {
                    float BrushStrength = CalculateBrushStrength(HalfWidth, FMath::Clamp<float>(RadialStrength, 0., 1.) * HalfWidth, (C-B).Size2D(), FalloffType);
                    if (bRelative)
                        Vertices[idx].Z += FMath::Lerp<float>(0., B.Z, BrushStrength);
                    else
                        Vertices[idx].Z = FMath::Lerp<float>(0., B.Z, BrushStrength);
                    UpdateZBounds(Vertices[idx].Z);
                }
            }
        }
    }
    UpdateNormals(Bounds);
}

void AAutoSceneGenLandscape::PostSculptUpdate()
{
    LandscapeMesh->UpdateMeshSection_LinearColor(0, Vertices, Normals, UV0, TArray<FLinearColor>(), TArray<FProcMeshTangent>());
    UE_LOG(LogASG, Display, TEXT("Performed post-sculpt update on landscape."));
}

void AAutoSceneGenLandscape::ResetToBaseMesh()
{
    Tangents.Empty();
    for (int32 i = 0; i < Vertices.Num(); i++)
    {
        Vertices[i].Z = GetActorLocation().Z;
        Normals[i] = FVector(0., 0., 1.);
    }
    LowerLeftCorner = GetActorLocation();
    BoundingBox.Min = GetActorLocation();
    BoundingBox.Max = GetActorLocation() + FVector(LandscapeSize, LandscapeSize, 0.);
    LandscapeMesh->UpdateMeshSection_LinearColor(0, Vertices, Normals, UV0, TArray<FLinearColor>(), TArray<FProcMeshTangent>());
}

void AAutoSceneGenLandscape::SetMaterial(UMaterialInterface* NewMaterial)
{
    LandscapeMesh->SetMaterial(0, NewMaterial);
    AnnotationComponent->UpdateDefaultMeshMaterials();
}

void AAutoSceneGenLandscape::ClearVertexArrays()
{
    Vertices.Empty();
    Triangles.Empty();
    Normals.Empty();
    Tangents.Empty();
    UV0.Empty();
    VertexColors.Empty();
}

void AAutoSceneGenLandscape::UpdateZBounds(float NewZ)
{
    BoundingBox.Min.Z = FMath::Min<float>(BoundingBox.Min.Z, NewZ);
    BoundingBox.Max.Z = FMath::Max<float>(BoundingBox.Max.Z, NewZ);
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

    // Remember to add vertices in CCW order
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
    NewTriangles.Emplace(i_bc);
    NewTriangles.Emplace(i_ca);
    NewTriangles.Emplace(i_ab);
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
    // return x*x*(3. - 2.*x); // This is what Unreal uses
    return 0.5 * FMath::Cos(PI * (x - 1.)) + 0.5; // Is one period of cosine smoother?
}

float AAutoSceneGenLandscape::CalculateBrushStrength(float BrushRadius, float EffectiveRadius, float Distance, uint8 FalloffType)
{
    if (FalloffType == ELandscapeFalloff::Linear)
        return CalculateLinearBrushStrength(BrushRadius, EffectiveRadius, Distance);
    else if (FalloffType == ELandscapeFalloff::Smooth)
        return CalculateSmoothBrushStrength(BrushRadius, EffectiveRadius, Distance);
    else
        return CalculateSmoothBrushStrength(BrushRadius, EffectiveRadius, Distance);
}

FVector AAutoSceneGenLandscape::GetVertexNormal(FIntPoint V)
{
    int32 MaxIdx = (int32)FMath::Pow(2, NumSubdivisions);
    V.X = FMath::Clamp<int32>(V.X, 0, MaxIdx);
    V.Y = FMath::Clamp<int32>(V.Y, 0, MaxIdx);

    int32 idx = VertexGridMap[V.X][V.Y];
    FVector N;

    // NOTE: Operator precedence
    // A^B + C^D is NOT equal to (A^B) + (C^D) becasue + takes precedence over ^
    // A^B + C^D is equal to A^(B + C)^D

    if (V.X == 0 && V.Y == 0) // Lower left corner
    {
        int32 iU = VertexGridMap[V.X+1][V.Y];
        int32 iR = VertexGridMap[V.X][V.Y+1];
        N = (Vertices[iU] - Vertices[idx])^(Vertices[iR] - Vertices[idx]);
    }
    else if (V.X == MaxIdx && V.Y == 0) // Upper left corner
    {
        int32 iR = VertexGridMap[V.X][V.Y+1];
        int32 iRD = VertexGridMap[V.X-1][V.Y+1];
        int32 iD = VertexGridMap[V.X-1][V.Y];
        N =  (Vertices[iR] - Vertices[idx])^(Vertices[iRD] - Vertices[idx]);
        N += (Vertices[iRD] - Vertices[idx])^(Vertices[iD] - Vertices[idx]);
    }
    else if (V.X == 0 && V.Y == MaxIdx) // Lower right corner
    {
        int32 iL = VertexGridMap[V.X][V.Y-1];
        int32 iLU = VertexGridMap[V.X+1][V.Y-1];
        int32 iU = VertexGridMap[V.X+1][V.Y];
        N =  (Vertices[iL] - Vertices[idx])^(Vertices[iLU] - Vertices[idx]);
        N += (Vertices[iLU] - Vertices[idx])^(Vertices[iU] - Vertices[idx]);
    }
    else if (V.X == MaxIdx && V.Y == MaxIdx) // Upper right corner
    {
        int32 iD = VertexGridMap[V.X-1][V.Y];
        int32 iL = VertexGridMap[V.X][V.Y-1];
        N = (Vertices[iD] - Vertices[idx])^(Vertices[iL] - Vertices[idx]);
    }
    else if (V.X == 0) // Bottom edge
    {
        int32 iL = VertexGridMap[V.X][V.Y-1];
        int32 iLU = VertexGridMap[V.X+1][V.Y-1];
        int32 iU = VertexGridMap[V.X+1][V.Y];
        int32 iR = VertexGridMap[V.X][V.Y+1];
        N =  (Vertices[iL] - Vertices[idx])^(Vertices[iLU] - Vertices[idx]);
        N += (Vertices[iLU] - Vertices[idx])^(Vertices[iU] - Vertices[idx]);
        N += (Vertices[iU] - Vertices[idx])^(Vertices[iR] - Vertices[idx]);
    }
    else if (V.Y == 0) // Left edge
    {
        int32 iU = VertexGridMap[V.X+1][V.Y];
        int32 iR = VertexGridMap[V.X][V.Y+1];
        int32 iRD = VertexGridMap[V.X-1][V.Y+1];
        int32 iD = VertexGridMap[V.X-1][V.Y];
        N =  (Vertices[iU] - Vertices[idx])^(Vertices[iR] - Vertices[idx]);
        N += (Vertices[iR] - Vertices[idx])^(Vertices[iRD] - Vertices[idx]); 
        N += (Vertices[iRD] - Vertices[idx])^(Vertices[iD] - Vertices[idx]);
    }
    else if (V.X == MaxIdx) // Upper edge
    {
        int32 iR = VertexGridMap[V.X][V.Y+1];
        int32 iRD = VertexGridMap[V.X-1][V.Y+1];
        int32 iD = VertexGridMap[V.X-1][V.Y];
        int32 iL = VertexGridMap[V.X][V.Y-1];
        N =  (Vertices[iR] - Vertices[idx])^(Vertices[iRD] - Vertices[idx]); 
        N += (Vertices[iRD] - Vertices[idx])^(Vertices[iD] - Vertices[idx]);
        N += (Vertices[iD] - Vertices[idx])^(Vertices[iL] - Vertices[idx]);
    }
    else if (V.Y == MaxIdx) // Right edge
    {
        int32 iD = VertexGridMap[V.X-1][V.Y];
        int32 iL = VertexGridMap[V.X][V.Y-1];
        int32 iLU = VertexGridMap[V.X+1][V.Y-1];
        int32 iU = VertexGridMap[V.X+1][V.Y];
        N =  (Vertices[iD] - Vertices[idx])^(Vertices[iL] - Vertices[idx]);
        N += (Vertices[iL] - Vertices[idx])^(Vertices[iLU] - Vertices[idx]);
        N += (Vertices[iLU] - Vertices[idx])^(Vertices[iU] - Vertices[idx]);
    }
    else // Interior grid point
    {
        int32 iU = VertexGridMap[V.X+1][V.Y];
        int32 iR = VertexGridMap[V.X][V.Y+1];
        int32 iRD = VertexGridMap[V.X-1][V.Y+1];
        int32 iD = VertexGridMap[V.X-1][V.Y];
        int32 iL = VertexGridMap[V.X][V.Y-1];
        int32 iLU = VertexGridMap[V.X+1][V.Y-1];
        N =  (Vertices[iU] - Vertices[idx])^(Vertices[iR] - Vertices[idx]);
        N += (Vertices[iR] - Vertices[idx])^(Vertices[iRD] - Vertices[idx]);
        N += (Vertices[iRD] - Vertices[idx])^(Vertices[iD] - Vertices[idx]);
        N += (Vertices[iD] - Vertices[idx])^(Vertices[iL] - Vertices[idx]);
        N += (Vertices[iL] - Vertices[idx])^(Vertices[iLU] - Vertices[idx]);
        N += (Vertices[iLU] - Vertices[idx])^(Vertices[iU] - Vertices[idx]);
    }
    return N.GetSafeNormal();
}

void AAutoSceneGenLandscape::UpdateNormals(FIntRect Bounds)
{
    int32 MaxIdx = (int32)FMath::Pow(2, NumSubdivisions);
    FIntRect NormalBounds = Bounds + FIntRect(-1, -1, 1, 1);
    NormalBounds.Clip(FIntRect(0, 0, MaxIdx, MaxIdx));

    // Zero every vertex normal that needs recalculating
    for (int32 i = NormalBounds.Min.X; i <= NormalBounds.Max.X; i++)
        for (int32 j = NormalBounds.Min.Y; j <= NormalBounds.Max.Y; j++)
            Normals[VertexGridMap[i][j]] = FVector(0.);

    // Compute normal for each face and add normal to each vertex
    for (int32 i = NormalBounds.Min.X; i < NormalBounds.Max.X; i++)
    {
        for (int32 j = NormalBounds.Min.Y; j < NormalBounds.Max.Y; j++)
        {
            int32 ia = VertexGridMap[i][j]; // LL
            int32 ib = VertexGridMap[i][j+1]; // LR
            int32 ic = VertexGridMap[i+1][j]; // UL
            int32 id = VertexGridMap[i+1][j+1]; // UR

            // Lower triangle
            FVector NV = (Vertices[ic] - Vertices[ia]) ^ (Vertices[ib] - Vertices[ia]);
            Normals[ia] += NV;
            Normals[ib] += NV;
            Normals[ic] += NV;

            // Upper triangle
            NV = (Vertices[ib] - Vertices[id]) ^ (Vertices[ic] - Vertices[id]);
            Normals[id] += NV;
            Normals[ic] += NV;
            Normals[ib] += NV;
        }
    }

    // Edges of NormalBounds
    if (NormalBounds.Min.X > 0) // Compute normals for bottom edge of NormalBounds if not coincident with bottom grid edge
    {
        for (int32 j = NormalBounds.Min.Y; j <= NormalBounds.Max.Y; j++)
        {
            int32 idx = VertexGridMap[NormalBounds.Min.X][j];
            Normals[idx] = GetVertexNormal(FIntPoint(NormalBounds.Min.X, j));
        }
    }
    if (NormalBounds.Max.X < MaxIdx) // Compute normals for upper edge of NormalBounds if not coincident with upper grid edge
    {
        for (int32 j = NormalBounds.Min.Y; j <= NormalBounds.Max.Y; j++)
        {
            int32 idx = VertexGridMap[NormalBounds.Max.X][j];
            Normals[idx] = GetVertexNormal(FIntPoint(NormalBounds.Max.X, j));
        }
    }
    if (NormalBounds.Min.Y > 0) // Compute normals for left edge of NormalBounds if not coincident with left grid edge
    {
        for (int32 i = NormalBounds.Min.X; i <= NormalBounds.Max.X; i++)
        {
            int32 idx = VertexGridMap[i][NormalBounds.Min.Y];
            Normals[idx] = GetVertexNormal(FIntPoint(i, NormalBounds.Min.Y));
        }
    }
    if (NormalBounds.Max.Y < MaxIdx) // Compute normals for left edge of NormalBounds if not coincident with left grid edge
    {
        for (int32 i = NormalBounds.Min.X; i <= NormalBounds.Max.X; i++)
        {
            int32 idx = VertexGridMap[i][NormalBounds.Max.Y];
            Normals[idx] = GetVertexNormal(FIntPoint(i, NormalBounds.Max.Y));
        }
    }

    // Corners of NormalBounds
    int32 idx;
    idx = VertexGridMap[NormalBounds.Min.X][NormalBounds.Min.Y];
    Normals[idx] = GetVertexNormal(NormalBounds.Min); // Lower left corner

    idx = VertexGridMap[NormalBounds.Max.X][NormalBounds.Min.Y];
    Normals[idx] = GetVertexNormal(FIntPoint(NormalBounds.Max.X, NormalBounds.Min.Y)); // Upper left corner

    idx = VertexGridMap[NormalBounds.Max.X][NormalBounds.Max.Y];
    Normals[idx] = GetVertexNormal(NormalBounds.Max); // Upper right corner

    idx = VertexGridMap[NormalBounds.Min.X][NormalBounds.Max.Y];
    Normals[idx] = GetVertexNormal(FIntPoint(NormalBounds.Min.X, NormalBounds.Max.Y)); // Lower right corner

    // Normalize all newly calculated vectors
    for (int32 i = NormalBounds.Min.X; i <= NormalBounds.Max.X; i++)
        for (int32 j = NormalBounds.Min.Y; j <= NormalBounds.Max.Y; j++)
            Normals[VertexGridMap[i][j]] = Normals[VertexGridMap[i][j]].GetSafeNormal();
}