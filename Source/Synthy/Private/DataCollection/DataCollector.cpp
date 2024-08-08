// Fill out your copyright notice in the Description page of Project Settings.


#include "DataCollection/DataCollector.h"

#include "Async/Async.h" // Include this header for asynchronous tasks
#include "EngineUtils.h"
#include "Components/SplineComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/LineBatchComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/SceneCaptureComponent2D.h"
#include "NavigationSystemTypes.h"
#include "Modules/ModuleManager.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Engine/Engine.h"
#include "Editor/UnrealEd/Public/Editor.h"
#include "Editor.h"
void FindAllLineBatchComponents(UWorld* World, TArray<ULineBatchComponent*>& LineBatchComponents)
{
	if (!World)
	{
		return;
	}

	for (TActorIterator<AActor> ActorItr(World); ActorItr; ++ActorItr)
	{
		AActor* Actor = *ActorItr;
		if (Actor)
		{
			TArray<UActorComponent*> Components = Actor->K2_GetComponentsByClass(ULineBatchComponent::StaticClass());
			for (UActorComponent* Component : Components)
			{
				if (ULineBatchComponent* LineBatchComponent = Cast<ULineBatchComponent>(Component))
				{
					LineBatchComponents.Add(LineBatchComponent);
				}
			}
		}
	}
}

FMatrix CalculateIntrinsicMatrix(UCineCameraComponent* CineCamera)
{
	if (!CineCamera)
	{
		UE_LOG(LogTemp, Error, TEXT("CineCameraComponent is null."));
		return FMatrix::Identity;
	}

	// Get the focal length in mm
	float FocalLength = CineCamera->CurrentFocalLength;

	// Get the sensor dimensions in mm
	float SensorWidth = CineCamera->Filmback.SensorWidth;
	float SensorHeight = CineCamera->Filmback.SensorHeight;

	// Get the image dimensions in pixels (assuming default aspect ratio here, adjust as necessary)
	float ImageWidth = 1920;
	float ImageHeight = 1080;

	// Calculate focal lengths in pixels
	float Fx = (FocalLength / SensorWidth) * ImageWidth;
	float Fy = (FocalLength / SensorHeight) * ImageHeight;

	// Principal point (assuming center of the image)
	float Cx = ImageWidth / 2.0f;
	float Cy = ImageHeight / 2.0f;

	// Create the intrinsic matrix
	FMatrix IntrinsicMatrix = FMatrix::Identity;
	IntrinsicMatrix.M[0][0] = Fx;
	IntrinsicMatrix.M[1][1] = Fy;
	IntrinsicMatrix.M[0][2] = Cx;
	IntrinsicMatrix.M[1][2] = Cy;
	IntrinsicMatrix.M[2][2] = 1.0f;

	return IntrinsicMatrix;
}

FVector ADataCollector::GetHeightAtDistanceAlongSpline(FTransform TransformAtSplineDistance)
{
	FVector NewLocation = TransformAtSplineDistance.GetLocation();

	FVector Offset(0.0f, 0.0f, -100000.0f);
	FVector LineTraceEnd = NewLocation + Offset;

	FHitResult HitResult;
	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(this);

	GetWorld()->LineTraceSingleByChannel(HitResult, NewLocation, LineTraceEnd, ECC_WorldStatic, CollisionParams);

	if (HitResult.bBlockingHit)
	{
		return HitResult.Location;
	}
	return  NewLocation;
}

// Sets default values
ADataCollector::ADataCollector()
{
	ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;


	int height = 720;
	int width = 1280;
	m_DefaultSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultSceneComponent"));
	RootComponent = m_DefaultSceneComponent;


	m_DebugCameraComponent = CreateDefaultSubobject<UCineCameraComponent>(TEXT("CineCameraComponent2"));
	m_DebugCameraComponent->SetupAttachment(m_DefaultSceneComponent);
	m_DebugCameraComponent->SetRelativeLocation(FVector(-200, 0, 0));

	// === handle camera rendering

	m_RenderTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("RenderTarget"));
	m_RenderTarget->InitAutoFormat(width,height); // Set the resolution you want
	m_NoDebugRenderTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("DebugRenderTarget"));
	m_NoDebugRenderTarget->InitAutoFormat(width,height); // Set the resolution you want

	m_SceneCaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCaptureComponent"));
	m_SceneCaptureComponent->SetupAttachment(m_DebugCameraComponent);

	m_NoDebugSceneCaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(
		TEXT("DebugSceneCaptureComponent"));
	
	m_SceneCaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
	m_NoDebugSceneCaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
	m_NoDebugSceneCaptureComponent->HideActorComponents(this,true);
	//m_NoDebugSceneCaptureComponent->SetupAttachment(m_CameraComponent);
	m_NoDebugSceneCaptureComponent->SetupAttachment(m_DebugCameraComponent);
	//m_NoDebugSceneCaptureComponent->HiddenActors.Add(this);
	//static ConstructorHelpers::FObjectFinder<UMaterial> BaseMaterial(TEXT("Material'/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial'"));
	static ConstructorHelpers::FObjectFinder<UMaterial> BaseMaterial(TEXT("Material'/Synthy/M_TrajectoryPoints.M_TrajectoryPoints'"));
	
	if (BaseMaterial.Succeeded())
	{
		int32 RedValue = 255;
		int32 GreenValue =0;
		int32 BlueValue = 255;

		// Convert the RGB values to a range of [0, 1]
		float Red = RedValue / 255.0f;
		float Green = GreenValue / 255.0f;
		float Blue = BlueValue / 255.0f;	
		UnlitWhiteMaterial = UMaterialInstanceDynamic::Create(BaseMaterial.Object, this);
		UnlitWhiteMaterial->SetScalarParameterValue(FName("Roughness"), 0.0f);
		UnlitWhiteMaterial->SetScalarParameterValue(FName("Metallic"), 0.0f);
		//UnlitWhiteMaterial->SetVectorParameterValue(FName("BaseColor"), FLinearColor::Red);
		FLinearColor ExactColor(Red, Green, Blue);

		// Set the "BaseColor" parameter of the material

		// Make it unlit
		UnlitWhiteMaterial->SetScalarParameterValue(FName("ShadingModel"), 0); // 0 means Unlit
		
		UnlitWhiteMaterial->SetVectorParameterValue(FName("EmissiveColor"), ExactColor);
	}	
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Cube"));

	if (SphereMesh.Succeeded())
	{
		PointMesh = SphereMesh.Object;
	}
}
void ADataCollector::CreatePath()
{
	if (DebugPoints.Num() < 2) return;

	// Create points
	for (int32 i = 0; i < DebugPoints.Num(); ++i)
	{
		UStaticMeshComponent* Point = NewObject<UStaticMeshComponent>(m_SplineActor);
		Point->SetStaticMesh(PointMesh);
		Point->SetRelativeScale3D(FVector(0.2f));
		Point->RegisterComponent();
		Point->SetWorldLocation(DebugPoints[i]);

		// Disable shadows and light interaction
		Point->bCastStaticShadow = false;
		Point->bCastDynamicShadow = false;
		Point->CastShadow = false;
		Point->SetMobility(EComponentMobility::Movable);

		// Set material to be unlit
		Point->SetMaterial(0, UnlitWhiteMaterial);

		// Ensure the point does not affect lighting
		Point->SetVisibility(true);
		Point->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Point->SetReceivesDecals(false);
		Point->bAffectDynamicIndirectLighting = false;
		Point->bAffectDistanceFieldLighting = false;
		Point->bVisibleInRayTracing = false;
		//Point->bRenderInMainPass = false;
		//Point->bRenderInDepthPass = false;

		// Additional settings
		Point->SetCastShadow(false); // Ensure this setting is applied
		Point->MarkRenderStateDirty(); // Update render state
		Point->MarkRenderDynamicDataDirty(); // Ensure dynamic data is updated
		Point->SetRenderCustomDepth(true);
		Point->SetCustomDepthStencilValue(55);
		PathPoints.Add(Point);
	}

}
// Called when the game starts or when spawned
void ADataCollector::BeginPlay()
{
	Super::BeginPlay();

	m_Spline = m_SplineActor->m_Spline;
	m_Spline->bDrawDebug = false; 
	m_Spline->SetHiddenInGame(true);
	
	
	m_NoDebugSceneCaptureComponent->TextureTarget = m_NoDebugRenderTarget;
	m_NoDebugSceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR; // this was ldr
	m_NoDebugSceneCaptureComponent->bCaptureEveryFrame =true;
	m_NoDebugSceneCaptureComponent->bCaptureOnMovement =true;
	m_NoDebugSceneCaptureComponent->bAlwaysPersistRenderingState = true;
	m_NoDebugSceneCaptureComponent->MaxViewDistanceOverride = -1.0f;
	m_NoDebugSceneCaptureComponent->PostProcessSettings =m_DebugCameraComponent->PostProcessSettings;
	m_NoDebugSceneCaptureComponent->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
	m_NoDebugSceneCaptureComponent->PostProcessSettings.bOverride_IndirectLightingColor = true;
	m_NoDebugSceneCaptureComponent->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
	m_NoDebugSceneCaptureComponent->PostProcessSettings.bOverride_ReflectionMethod = true;
	
	m_NoDebugSceneCaptureComponent->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::Lumen;
	m_NoDebugSceneCaptureComponent->PostProcessSettings.ReflectionMethod = EReflectionMethod::Lumen;
	m_NoDebugSceneCaptureComponent->PostProcessSettings.IndirectLightingIntensity = 1.0f;
	
	m_NoDebugSceneCaptureComponent->bUseRayTracingIfEnabled = true;
	m_NoDebugSceneCaptureComponent->RegisterComponent();
	
	m_SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorHDR;
	m_SceneCaptureComponent->bCaptureEveryFrame =true;
	m_SceneCaptureComponent->bCaptureOnMovement =true;
	m_SceneCaptureComponent->bAlwaysPersistRenderingState = true;
	m_SceneCaptureComponent->CompositeMode = ESceneCaptureCompositeMode::SCCM_Overwrite;
	m_SceneCaptureComponent->TextureTarget = m_RenderTarget;
	m_SceneCaptureComponent->ShowFlags.SetTemporalAA(true);
	m_SceneCaptureComponent->ShowFlags.SetTonemapper(false);
	m_SceneCaptureComponent->AddOrUpdateBlendable(m_PostProcessMaterial);
	
	m_SceneCaptureComponent->RegisterComponent();
	
	
	m_SplineLength = m_Spline->GetSplineLength();
	m_DistanceAlongSpline = 0.0f;
	m_DistanceSinceLastCapture = 0;
	
	float distBetweenCaptures = 50;
	float currentDistance = 0;
	while (currentDistance < m_SplineLength)
	{
		FTransform transform = m_Spline->GetTransformAtDistanceAlongSpline(
			currentDistance, ESplineCoordinateSpace::World);
		FVector p = GetHeightAtDistanceAlongSpline(transform);
		p.Z += 5;
		DebugPoints.Add(p);
		
		UE_LOG(LogTemp, Warning, TEXT("%f,%f,%f"),p.X,p.Y,p.Z);
		currentDistance += distBetweenCaptures;
	}
	for (int i = 0; i < m_Spline->GetNumberOfSplinePoints(); i++)
	{
		SetSplinePoint(i);
	}
	
	CreatePath();
	
	
	m_NoDebugSceneCaptureComponent->HiddenActors.Add(m_SplineActor);
}


void ADataCollector::SetSplinePoint(int index)
{
	FTransform transform = m_Spline->GetTransformAtSplinePoint(index, ESplineCoordinateSpace::World);

	FVector Location = GetHeightAtDistanceAlongSpline(transform);
	Location.Z += m_HeightOffset;
	//FVector Location = transform.GetLocation();
	m_Spline->SetLocationAtSplinePoint(index, Location, ESplineCoordinateSpace::World);


	FVector Tangent = m_Spline->GetTangentAtSplinePoint(index, ESplineCoordinateSpace::World);
	//Spline->SetTangentAtSplinePoint(index, Tangent, ESplineCoordinateSpace::World, true);
	m_Spline->UpdateSpline();
}

void ADataCollector::VisualizePointsAndLines(const TArray<FVector>& Points)
{
	if (Points.Num() < 2)
	{
		UE_LOG(LogTemp, Warning, TEXT("Not enough points to draw lines."));
		return;
	}

	// Loop through each point and draw a sphere at the location
	for (const FVector& Point : Points)
	{
		//DrawDebugSphere(GetWorld(), Point, 20.0f, 12, FColor::White, true, -1.0f, 0, 2.0f);
		DrawDebugBox(GetWorld(), Point, FVector(2, 2, 2), FColor::White, true, -1.0f, 0, 2.0f);
	}

	// Loop through each pair of points and draw a line connecting them
	for (int32 i = 0; i < Points.Num() - 1; ++i)
	{
		DrawDebugLine(GetWorld(), Points[i], Points[i + 1], FColor::White, true, -1.0f, 0, 2.0f);
	}
}

void ADataCollector::SavePositionAndOrientation(const FVector& Position, const FQuat& Orientation)
{
	FString FilePath = FPaths::ProjectSavedDir() / TEXT("Screenshots/positions_orientations.csv");

	// Create the string to save
	FString SaveText = FString::Printf(TEXT("%f,%f,%f,%f,%f,%f,%f\n"),
	                                   Position.X / 10, Position.Y / 10, Position.Z / 10,
	                                   Orientation.X, Orientation.Y, Orientation.Z, Orientation.W);

	// Save to the file
	FFileHelper::SaveStringToFile(SaveText, *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(),
	                              FILEWRITE_Append);
}

void ADataCollector::CaptureSceneData(UTextureRenderTarget2D* target)
{
	if (m_RenderTarget)
	{
		UE_LOG(LogTemp, Log, TEXT("render target valid trying to save"));
		// Read pixels from the render target
		TArray<FColor> Bitmap;
		FRenderTarget* RenderTargetResource = target->GameThread_GetRenderTargetResource();
		RenderTargetResource->ReadPixels(Bitmap);

		// Save the image
		SaveImage(Bitmap, m_RenderTarget->SizeX, m_RenderTarget->SizeY);

		//m_DistanceSinceLastCapture = 0;
	}
}

// Called every frame
void ADataCollector::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	m_SplineLength = m_Spline->GetSplineLength();

	float screenWidth = m_NoDebugRenderTarget->SizeX;
	float correctedFOV = screenWidth / FMath::Tan(FMath::DegreesToRadians(m_NoDebugSceneCaptureComponent->FOVAngle *0.5f));

	FVector currentLocation = GetActorLocation();
	float currentDistance = m_Spline->GetDistanceAlongSplineAtLocation(currentLocation, ESplineCoordinateSpace::World);
	
	IStreamingManager::Get().AddViewInformation(
	m_NoDebugSceneCaptureComponent->GetComponentLocation(),screenWidth,correctedFOV
	);
	if (m_mode == 0)
	{
		
		
		const float distanceToMove = m_DistancePerSecond * DeltaTime;
		//m_DistanceAlongSpline += distanceToMove;
		m_DistanceSinceLastCapture += distanceToMove;
		m_DistanceAlongSpline = currentDistance + distanceToMove;
			
		UE_LOG(LogTemp, Log, TEXT("Rig at %f out of %f along spline"),m_DistanceAlongSpline, m_Spline->GetSplineLength());
		if (m_DistanceAlongSpline >= m_Spline->GetSplineLength()){

			if (GIsEditor && GEditor)
			{
				if (GEditor->PlayWorld)
				{
					// Stop the game
					GEditor->RequestEndPlayMap();
				}
				
			}
		
			return;
			
		}


		FTransform NewTransform = m_Spline->GetTransformAtDistanceAlongSpline(
			m_DistanceAlongSpline, ESplineCoordinateSpace::World);
		FVector NewLocation = GetHeightAtDistanceAlongSpline(NewTransform);
		NewLocation.Z += m_HeightOffset;
		NewTransform.SetLocation(NewLocation);

		// Get the current location of the actor
		FVector CurrentLocation = GetActorLocation();
		FVector InterpolatedLocation = FMath::Lerp(CurrentLocation, NewTransform.GetLocation(), m_LerpSpeed * DeltaTime);
		NewTransform.SetLocation(InterpolatedLocation);
		m_DefaultSceneComponent->SetWorldTransform(NewTransform);
		if (m_DistanceSinceLastCapture >= m_CaptureDistanceEvery)

		{
			m_mode = 1;
			//return;
		}
	}

		m_SceneCaptureComponent->CaptureScene();
		m_NoDebugSceneCaptureComponent->CaptureScene();

	if (m_mode == 1)
	{
		UE_LOG(LogTemp, Log, TEXT("saving image"));

		FVector Position = m_DefaultSceneComponent->GetComponentTransform().GetLocation();
		FQuat Orientation = m_DefaultSceneComponent->GetComponentTransform().GetRotation();
		SavePositionAndOrientation(Position, Orientation);

		CaptureSceneData(m_NoDebugRenderTarget);
		m_mode += 1;
		
		CaptureSceneData(m_RenderTarget);

		m_mode = 2;
		m_sample += 1;
		m_DistanceSinceLastCapture = 0;
	}
	m_mode += 2;
	if (m_mode >= 2)
	{
		m_mode = 0;
	}
}

/*void ADataCollector::SaveImage(const TArray<FColor>& Bitmap, int32 Width, int32 Height)
{
	//FString FilePath = FPaths::ProjectSavedDir() / TEXT("Screenshots/") / FDateTime::Now().ToString() + TEXT(".png");
	FString FilePath = FPaths::ProjectSavedDir() / TEXT("Screenshots/") / FString::Printf(
		TEXT("%d_%d.png"), m_sample, m_mode);

	// Use the image wrapper module to save the image as PNG
	if (ImageWrapper.IsValid() && ImageWrapper->SetRaw(Bitmap.GetData(), Bitmap.Num() * sizeof(FColor), Width, Height,
	                                                   ERGBFormat::BGRA, 8))
	{
		const TArray64<uint8>& CompressedData64 = ImageWrapper->GetCompressed(100);

		// Convert TArray64<uint8> to TArray<uint8>
		TArray<uint8> CompressedData;
		CompressedData.Append(CompressedData64.GetData(), CompressedData64.Num());

		// Save the compressed data to a file
		FFileHelper::SaveArrayToFile(CompressedData, *FilePath);
	}
}*/
void ADataCollector::SaveImage(const TArray<FColor>& Bitmap, int32 Width, int32 Height)
{

	if(!m_ShouldSave)
		return;
	// Copy the parameters to capture them in the lambda
	TArray<FColor> BitmapCopy = Bitmap;
	int32 WidthCopy = Width;
	int32 HeightCopy = Height;
	int32 SampleCopy = m_sample;
	int32 ModeCopy = m_mode;
	TSharedPtr<IImageWrapper> ImageWrapperCopy = ImageWrapper; // Make a copy of ImageWrapper to capture in the lambda

	// Run the task on a background thread
		FString FilePath = FPaths::ProjectSavedDir() / FString::Printf(TEXT("Screenshots/%d/"),m_mode) / FString::Printf(TEXT("%d_%d.png"), SampleCopy, ModeCopy);

		// Use the image wrapper module to save the image as PNG
		if (ImageWrapperCopy.IsValid() && ImageWrapperCopy->SetRaw(BitmapCopy.GetData(), BitmapCopy.Num() * sizeof(FColor), WidthCopy, HeightCopy,
																   ERGBFormat::BGRA, 8))
		{
			const TArray64<uint8>& CompressedData64 = ImageWrapperCopy->GetCompressed(100);

			// Convert TArray64<uint8> to TArray<uint8>
			TArray<uint8> CompressedData;
			CompressedData.Append(CompressedData64.GetData(), CompressedData64.Num());

	Async(EAsyncExecution::Thread, [CompressedData,FilePath]()
	{
			// Save the compressed data to a file
			FFileHelper::SaveArrayToFile(CompressedData, *FilePath);
			
	});
		}
}

// Called to bind functionality to input
void ADataCollector::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}
