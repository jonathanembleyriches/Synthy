#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CustomSegmentationAnnotator.generated.h"

UCLASS()
class SYNTHY_API ACustomSegmentationAnnotator : public AActor
{
    GENERATED_BODY()

public:
    // Constructor
    ACustomSegmentationAnnotator();

protected:
    // Called when the game starts or the actor is spawned
    virtual void BeginPlay() override;

public:
    // Function to apply custom depth stencil values to all unique meshes in the scene
    UFUNCTION(BlueprintCallable, Category = "Custom Depth")
    void AssignDepthStencils();

private:
    // Counter for unique stencil values
    
    UPROPERTY()
    uint8 CurrentStencilValue = 0;

    // Map to track unique meshes and their stencil values
    
    UPROPERTY()
    TMap<UStaticMesh*, uint8> StaticMeshStencilMap;
    UPROPERTY()
    TMap<USkeletalMesh*, uint8> SkeletalMeshStencilMap;

    // Helper function to process a static mesh component
    void ProcessStaticMeshComponent(UStaticMeshComponent* MeshComponent);

    // Helper function to process a skeletal mesh component
    void ProcessSkeletalMeshComponent(USkeletalMeshComponent* MeshComponent);
};
