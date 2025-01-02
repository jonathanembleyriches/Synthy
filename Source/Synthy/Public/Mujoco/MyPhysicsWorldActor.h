// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <functional>


#include "Communication/RosManager.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <mujoco/mujoco.h>
#include "Utils/MJHelper.h"
#include "MyPhysicsWorldActor.generated.h"

UCLASS()
class SYNTHY_API AMyPhysicsWorldActor : public AActor {
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    AMyPhysicsWorldActor();
    ~AMyPhysicsWorldActor();

    // float Multiplier = 100.0f;

    // Called every frame
    virtual void Tick(float DeltaTime) override;
    virtual void BeginDestroy() override;

    float Multiplier = 100.0f; // Adjust the multiplier as needed
    struct PhysicsObject {
        FString ObjectName; // class name for cache
        AActor* Actor;
        USceneComponent* SceneComponent;
        UStaticMeshComponent* MeshComponent;
        FTransform Transform;
        FRotator Rotation;
        FVector Scale;
        FString ObjectPathOrPrimitive;
        int SubObjectCount;
        bool Duplicate;

        bool DrawDebug = false;

        FVector OffsetPos;
        FQuat OffsetQuat;
        bool Static;
        bool RobotPart;
    };

    bool m_PublishDataToRos = false;
    struct MujocoGeomData{

        mjtNum* pos;
        mjtNum* mat;
        FVector Position;
        FVector RefPos;

        FQuat LocalQuatUE;
        FQuat GlobalQuatUE;
    };
    MujocoGeomData GetMujocoGeomData(int geomId);

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
    TArray<AActor*> PhysicsStaticActors1;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
    TArray<AActor*> PhysicsDynamicActors1;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool m_ShowMujDebug = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool m_JointControlEnabled = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool bShouldStopTask=false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool m_ResetSim = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool m_PauseSim = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool bRecreateMujoco=false;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TMap<FString, float> m_ActuatorValues;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TMap<FString, float> m_JointValues;

    TMap<int, PhysicsObject*> m_MJRobotVisToPhysicsObject;
    TMap<int, PhysicsObject*> m_MJDynamicToPhysicsObject;


protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    void StopMJ();
    void ComputeMujocoUnrealSetup();

    UWorld* m_World;
    void SetupCameras();
    MJHelper::HeightFieldData HandleLandscapes();

void UpdateActuatorValuesFromKeyframe( const FString& keyframeName) ;
    void RunMujocoAsync();

    void DrawAllBodiesDebug();
    void SyncDynamic();


    void SetupActuatorAddresses();

    void SetupJointAddresses();

    void SetupRobotMJtoUE();


    void SyncMujBodyToUnreal(int body_id, PhysicsObject* p);

    void SetupMJActors(TArray<AActor*> Actors, bool RobotPart, bool ComplexMeshRequired, bool Static);


private: 

private: // UE
    //
    RosManager* m_RosManager;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureComponent;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureComponentDepth;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTargetRGB;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTargetDepth;

    AActor* m_GoalActor;

    FVector m_StartPos{0, 0, 0};
    FQuat m_Orientation{0, 0, 0, 1};

private: // muj
    float AccumulatedTime = 0.0f;
    mjModel* model;
    mjData* data;

    TMap<FString, int> m_JointAddressesMap;
    TMap<FString, int> m_ActuatorAddressesMap;

    UPROPERTY()
    FQuat m_CustomQuat = FQuat(0.9238795, -0.3826834, 0, 0);

    // ros stuff ==
    TMap<FString, TArray<PhysicsObject>> m_mjPhysicsObjects;
    TMap<FString, PhysicsObject> m_NameToPhysicsObject;
};
