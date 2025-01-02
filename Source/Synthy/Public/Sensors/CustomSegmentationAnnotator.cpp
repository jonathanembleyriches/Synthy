#include "Sensors/CustomSegmentationAnnotator.h"
#include "Engine/World.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Mujoco/MyPhysicsWorldActor.h"

// Constructor
ACustomSegmentationAnnotator::ACustomSegmentationAnnotator()
{
    // Default stencil value starts from 1 (0 is usually reserved for "no stencil")
    CurrentStencilValue = 1;
}

// Called when the game starts
void ACustomSegmentationAnnotator::BeginPlay()
{
    Super::BeginPlay();

    // Automatically assign stencils on BeginPlay
    AssignDepthStencils();
}

// Assign depth stencil values to all unique meshes in the scene
void ACustomSegmentationAnnotator::AssignDepthStencils()
{
    // Get all actors in the world
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Warning, TEXT("CustomSegmentationAnnotator: World is null, cannot assign depth stencils."));
        return;
    }

    for (TActorIterator<AActor> ActorItr(World); ActorItr; ++ActorItr)
    {
        AActor* Actor = *ActorItr;
        if (!Actor) continue;

        // Process Static Mesh Components
        TArray<UStaticMeshComponent*> StaticMeshComponents;
        Actor->GetComponents<UStaticMeshComponent>(StaticMeshComponents);
        for (UStaticMeshComponent* MeshComponent : StaticMeshComponents)
        {
            ProcessStaticMeshComponent(MeshComponent);
        }

        // Process Skeletal Mesh Components
        TArray<USkeletalMeshComponent*> SkeletalMeshComponents;
        Actor->GetComponents<USkeletalMeshComponent>(SkeletalMeshComponents);
        for (USkeletalMeshComponent* MeshComponent : SkeletalMeshComponents)
        {
            ProcessSkeletalMeshComponent(MeshComponent);
        }
    }
}

// Helper function to process static mesh components
void ACustomSegmentationAnnotator::ProcessStaticMeshComponent(UStaticMeshComponent* MeshComponent)
{
    if (!MeshComponent) return;

    UStaticMesh* StaticMesh = MeshComponent->GetStaticMesh();
    if (!StaticMesh) return;

    // Check if this mesh already has a stencil value assigned
    uint8* ExistingStencil = StaticMeshStencilMap.Find(StaticMesh);
    if (!ExistingStencil)
    {
        // Assign a new stencil value
        StaticMeshStencilMap.Add(StaticMesh, CurrentStencilValue);
        ExistingStencil = &CurrentStencilValue;
        CurrentStencilValue = (CurrentStencilValue + 1) % 256;
        if (CurrentStencilValue == 0) CurrentStencilValue = 1; // Skip 0
    }

    // Set the stencil value for this component
    MeshComponent->SetRenderCustomDepth(true);
    MeshComponent->SetCustomDepthStencilValue(*ExistingStencil);
}

// Helper function to process skeletal mesh components
void ACustomSegmentationAnnotator::ProcessSkeletalMeshComponent(USkeletalMeshComponent* MeshComponent)
{
    if (!MeshComponent) return;

    USkeletalMesh* SkeletalMesh = MeshComponent->SkeletalMesh;
    if (!SkeletalMesh) return;

    // Check if this mesh already has a stencil value assigned
    uint8* ExistingStencil = SkeletalMeshStencilMap.Find(SkeletalMesh);
    if (!ExistingStencil)
    {
        // Assign a new stencil value
        SkeletalMeshStencilMap.Add(SkeletalMesh, CurrentStencilValue);
        ExistingStencil = &CurrentStencilValue;
        CurrentStencilValue = (CurrentStencilValue + 1) % 256;
        if (CurrentStencilValue == 0) CurrentStencilValue = 1; // Skip 0
    }

    // Set the stencil value for this component
    MeshComponent->SetRenderCustomDepth(true);
    MeshComponent->SetCustomDepthStencilValue(*ExistingStencil);
}
