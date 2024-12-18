// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <functional>


#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <mujoco/mujoco.h>

#include "Utils/MJHelper.h"
#include "Utils/MeshUtils.h"
#include "Kismet/GameplayStatics.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/geometry_msgs/Point.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"
#include "ROSIntegration/Public/sensor_msgs/Imu.h"
#include "ROSIntegration/Public/sensor_msgs/JointState.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"

// #include "coacd.h"
// #include "model_obj.h"
// #include "config.h"
// #include "process.h"

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
    bool bRecreateMujoco=false;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TMap<FString, float> m_ActuatorValues;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TMap<FString, float> m_JointValues;

    TMap<int, PhysicsObject*> m_MJRobotVisToPhysicsObject;
    TMap<int, PhysicsObject*> m_MJDynamicToPhysicsObject;

    UFUNCTION(BlueprintCallable, Category = "ROS")
    void SetupRos(FString Addr);

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    void StopMJ();
    void ComputeMujocoUnrealSetup();
    void SubCallback_JointState(TSharedPtr<FROSBaseMsg> Msg);

    UWorld* m_World;
    void SetupCameras();
    MJHelper::HeightFieldData HandleLandscapes();
    void PublishJointState(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg, FROSTime RosTime);

    void PublishImu(TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg, FROSTime RosTime);
    void PublishContacts(TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> Msg, FROSTime RosTime);

    void PublishCamera(const TArray<FColor>& OutBMP, uint32 Width, uint32 Height, FROSTime RosTime);

    void PublishDepth(const TArray<FFloat16Color>& DepthData, uint32 Width, uint32 Height, FROSTime RosTime);

    void SetupJointStateSub();

    void RunMujocoAsync();
    void SubCallback_StartPos(TSharedPtr<FROSBaseMsg> Msg);

    void SetupStartPosSub();
    void DrawAllBodiesDebug();
    void SyncDynamic();

    void SubCallback_GoalPos(TSharedPtr<FROSBaseMsg> Msg);

    void SetupGoalPosSub();

    void SetupPublishers();

    void SetupActuatorAddresses();

    void SetupJointAddresses();

    void SetupRobotMJtoUE();

    void ReadAllSensorDataRos(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg,
                              TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg,
                              TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> TouchForceMsg, FROSTime& ROSTime);

    void SyncMujBodyToUnreal(int body_id, PhysicsObject* p);

    void SetupMJActors(TArray<AActor*> Actors, bool RobotPart, bool ComplexMeshRequired, bool Static);

    // void DecomposeMesh(FString ObjFileName) {
    //     coacd::Params params;
    //     coacd::Model coacdModel;
    //     const std::string StandardString = TCHAR_TO_UTF8(*ObjFileName);
    //     coacdModel.LoadOBJ(StandardString);
    //
    //     array<array<double, 3>, 3> rot;
    //     vector<double> bbox = coacdModel.Normalize();
    //     bool is_manifold = coacd::IsManifold(coacdModel);
    //     if (!is_manifold) {
    //         exit(0);
    //     }
    //     coacdModel.SaveOBJ(StandardString);
    //
    //     rot = coacdModel.PCA();
    //
    //     vector<coacd::Model> parts = coacd::Compute(coacdModel, params);
    //
    //     coacd::RecoverParts(parts, bbox, rot, params);
    //
    //     string objName = regex_replace(params.output_name, std::regex("wrl"), "obj");
    //     string wrlName = regex_replace(params.output_name, std::regex("obj"), "wrl");
    //
    //     coacd::SaveVRML(wrlName, parts, params);
    //     coacd::SaveOBJ(objName, parts, params);
    // }

private: // ROS
    //
    UPROPERTY()
    class UROSIntegrationGameInstance* ROSInst;

    UPROPERTY()
    UTopic* m_Topic_JointStateSub;

    UPROPERTY()
    UTopic* m_Topic_JointStatePub;

    UPROPERTY()
    UTopic* m_Topic_ImuPub;

    UPROPERTY()
    UTopic* m_Topic_CameraPub;

    UPROPERTY()
    UTopic* m_Topic_CameraDepthPub;

    UPROPERTY()
    UTopic* m_Topic_StartPosSub;

    UPROPERTY()
    UTopic* m_Topic_GoalPosSub;

    UPROPERTY()
    UTopic* m_Topic_ContactsPub;


private: // UE
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
