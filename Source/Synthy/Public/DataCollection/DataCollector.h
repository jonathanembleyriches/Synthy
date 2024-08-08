#pragma once

#include "CoreMinimal.h"
#include "Components/SplineComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/Material.h"

#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "MyDataSpline.h"
#include "Runtime/CinematicCamera/Public/CineCameraComponent.h"
#include "GameFramework/Pawn.h"
#include "DataCollector.generated.h"

UCLASS()
class SYNTHY_API ADataCollector : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	ADataCollector();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	UPROPERTY(EditAnywhere)
	AMyDataSpline* m_SplineActor;
	
	UPROPERTY(EditAnywhere)
	float m_CaptureDistanceEvery = 10;
	
	UPROPERTY(EditAnywhere, Category="Segmentation Setup")
	UMaterial* m_PostProcessMaterial = nullptr;
	
	UPROPERTY(EditAnywhere)
	bool m_ShouldSave = true;
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	void VisualizePointsAndLines(const TArray<FVector>& Points);
	void SavePositionAndOrientation(const FVector& Position, const FQuat& Orientation);
	void CaptureSceneData(UTextureRenderTarget2D* target);
	FVector GetHeightAtDistanceAlongSpline(FTransform TransformAtSplineDistance);
	void AddPointAtDistance(float Distance);
	void SetSplinePoint(int index);
	void SaveImage(const TArray<FColor>& Bitmap, int32 Width, int32 Height);

	
	UPROPERTY()
	UStaticMesh* PointMesh;


	UPROPERTY()
	TArray<UStaticMeshComponent*> PathPoints;
	
	
	UPROPERTY()
	USplineComponent* m_Spline;

	UPROPERTY()
	USceneComponent* m_DefaultSceneComponent;


	UPROPERTY(EditAnywhere)
	UCineCameraComponent* m_DebugCameraComponent;
	
	
	UPROPERTY()
	UTextureRenderTarget2D* m_NoDebugRenderTarget;

	UPROPERTY(EditAnywhere)
	USceneCaptureComponent2D* m_NoDebugSceneCaptureComponent;
	
	UPROPERTY()
	float m_SplineLength;
	
	UPROPERTY()
	float m_DistanceAlongSpline;
	
	UPROPERTY()
	float m_DistanceSinceLastCapture;

	UPROPERTY()
	int m_mode = 0;
	
	UPROPERTY()
	int m_sample = 0;

public:
	UPROPERTY()
	UMaterialInstanceDynamic* UnlitWhiteMaterial;	
void CreatePath();
	UPROPERTY()
	int test = 0;
	// camera stuff
	
	UPROPERTY()
	UTextureRenderTarget2D* m_RenderTarget;
	
	UPROPERTY()
	USceneCaptureComponent2D* m_SceneCaptureComponent;
	
	UPROPERTY(EditAnywhere)
	float m_HeightOffset = 50;

	UPROPERTY(EditAnywhere)
	float m_DistancePerSecond = 25;

	UPROPERTY(EditAnywhere)
	float m_LerpSpeed = 0.5f;


	TArray<FVector> DebugPoints;
	IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(
		FName("ImageWrapper"));
	TSharedPtr<IImageWrapper> ImageWrapper;
	

};
