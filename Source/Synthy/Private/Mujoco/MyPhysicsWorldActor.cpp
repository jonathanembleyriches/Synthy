// Fill out your copyright notice in the Descr

#include "AI/Navigation/NavCollisionBase.h"
#include "Animation/AnimSequenceHelpers.h"
#include "Async/Async.h"
#include "Chaos/ArrayCollection.h"
#include "Chaos/ArrayCollectionArray.h"
#include "Chaos/Core.h"
#include "Chaos/Particle/ObjectState.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "Chaos/Vector.h"
#include "Coacd/CoacdInterface.h"
#include "Communication/RosManager.h"
#include "Components/ShapeComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Containers/StringConv.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"
#include "Kismet/GameplayStatics.h"
#include "Landscape.h" // Include this to work with ALandscape
#include "Math/Float16Color.h"
#include "Mujoco/MyPhysicsWorldActor.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"
#include "Runtime/CinematicCamera/Public/CineCameraComponent.h"
#include "Textures/TextureAtlas.h"
#include "Utils/IO.h"
#include "Utils/XmlManager.h"
#include "coacd.h"
#include "zmq.hpp"
#include <mujoco/mujoco.h>
// #include "../src/logger.h"
// #include "../src/preprocess.h"
// #include "../src/process.h"
void AMyPhysicsWorldActor::EndPlay(const EEndPlayReason::Type EndPlayReason) {
    Super::EndPlay(EndPlayReason);

    // Perform custom cleanup
    if (EndPlayReason == EEndPlayReason::Destroyed) {
        UE_LOG(LogTemp, Warning, TEXT("Actor %s is being destroyed."), *GetName());
    }

    m_ResetSim = true;
    bShouldStopTask = true;
    UE_LOG(LogTemp, Warning, TEXT("Actor %s EndPlay called with reason: %d"), *GetName(), (int32)EndPlayReason);

    // TODO: we need to clean up the PhysicsObjects maps
    //
    //  m_NameToPhysicsObject.Add(NewPhysicsObject.ObjectName, NewPhysicsObject);
    //
    // TMap<int, PhysicsObject*> m_MJRobotVisToPhysicsObject;
    // TMap<int, PhysicsObject*> m_MJDynamicToPhysicsObject;
}
// Sets default values
AMyPhysicsWorldActor::AMyPhysicsWorldActor() {

    PrimaryActorTick.bCanEverTick = true;
}

AMyPhysicsWorldActor::~AMyPhysicsWorldActor() {}
void GetMuJoCoContactData(const mjModel* m, const mjData* d, TArray<FVector>& ContactLocations, TArray<FVector>& ContactForces) {
    // Clear the arrays
    ContactLocations.Empty();
    ContactForces.Empty();

    // Loop through all active contacts
    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& contact = d->contact[i];

        // Extract contact position in MuJoCo (world coordinates)
        FVector Location(static_cast<float>(contact.pos[0] * 100), static_cast<float>(contact.pos[1] * -100),
                         static_cast<float>(contact.pos[2] * 100));

        // Retrieve the contact force
        mjtNum contactForce[6]; // 6D force vector: 3 normal, 3 tangential
        mj_contactForce(m, d, i, contactForce);

        // Convert contact force to FVector
        FVector Force(static_cast<float>(contactForce[0]), static_cast<float>(contactForce[1] * -1),
                      static_cast<float>(contactForce[2]));

        // Add to output arrays
        ContactLocations.Add(Location);
        ContactForces.Add(Force);
    }
}
void VisualizeContactForces(UWorld* World, const TArray<FVector>& ContactLocations, const TArray<FVector>& ContactForces) {
    // Ensure the arrays are the same size
    if (ContactLocations.Num() != ContactForces.Num()) {
        UE_LOG(LogTemp, Error, TEXT("Contact locations and forces arrays are mismatched!"));
        return;
    }

    // Iterate through the contact data
    for (int32 i = 0; i < ContactLocations.Num(); ++i) {
        FVector Location = ContactLocations[i];
        FVector Force = ContactForces[i];

        // Scale the force for visualization
        float ForceScale = 1.0f; // Adjust this value as needed
        FVector ScaledForce = Force * ForceScale;

        // Draw the contact location as a sphere
        DrawDebugSphere(World, Location,
                        1.0f,          // Sphere radius
                        12,            // Number of segments
                        FColor::Green, // Color
                        false,         // Persistent lines
                        -1.0f          // Life time
        );

        // Draw the force as a line
        DrawDebugLine(World, Location, Location + ScaledForce,
                      FColor::Red, // Color
                      false,       // Persistent lines
                      -1.0f,       // Life time
                      0,           // Depth priority
                      0.5f         // Thickness
        );
    }
}
TArray<AActor*> GetAllActorsWithTag(UWorld* World, FName Tag) {
    TArray<AActor*> FoundActors;
    if (World) {
        UGameplayStatics::GetAllActorsWithTag(World, Tag, FoundActors);
    }
    return FoundActors;
}

void AMyPhysicsWorldActor::ComputeMujocoUnrealSetup() {

    // Iterate through all geoms
    for (int geomId = 0; geomId < model->ngeom; ++geomId) {
        if (model->geom_type[geomId] != mjGEOM_MESH)
            continue;

        MujocoGeomData geomData = GetMujocoGeomData(geomId);
        mjtNum* pos = geomData.pos;
        mjtNum* mat = geomData.mat;
        FVector Position = geomData.Position;
        FVector refPos = geomData.RefPos;
        FQuat LocalQuatUE = geomData.LocalQuatUE;
        FQuat GlobalQuatUE = geomData.GlobalQuatUE;

        int meshId = model->geom_dataid[geomId];

        if (model->mesh_graphadr[meshId] == -1) {
            continue; // No convex hull data available for this mesh
        }

        const char* geomName = mj_id2name(model, mjOBJ_GEOM, geomId);
        FString s = UTF8_TO_TCHAR(geomName);
        int32 SubIndex = s.Find(TEXT("_sub_"), ESearchCase::IgnoreCase, ESearchDir::FromStart);
        if (SubIndex != INDEX_NONE) {
            s = s.Left(SubIndex);
        }

        PhysicsObject* p = m_NameToPhysicsObject.Find(s);
        if (p == nullptr)
            continue;

        FQuat initialQuat = p->Rotation.Quaternion();
        FVector rp = refPos * Multiplier;
        rp.Y *= -1;

        FQuat OffsetQuat = initialQuat * LocalQuatUE.Inverse(); // this is working for static
        OffsetQuat = initialQuat * LocalQuatUE.Inverse() * GlobalQuatUE.Inverse();
        FVector OffsetPos = p->Transform.GetLocation() - rp;
        OffsetPos = -rp;

        p->OffsetPos = OffsetPos;
        p->OffsetQuat = OffsetQuat;
    }
}
void AMyPhysicsWorldActor::SetupMJActors(TArray<AActor*> Actors, bool RobotPart, bool ComplexMeshRequired, bool Static) {

    for (auto Actor : Actors) {

        TInlineComponentArray<UActorComponent*, 30> Components;
        Actor->GetComponents(UStaticMeshComponent::StaticClass(), Components);
        int MeshIndex = 0;
        UCineCameraComponent* CineCameraComp = Actor->FindComponentByClass<UCineCameraComponent>();

        if (CineCameraComp)
            continue;

        for (auto&& Comp : Components) {

            UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Comp);
            UStaticMesh* Mesh = SMC->GetStaticMesh();

            if (!Mesh)
                continue;

            UBodySetup* BodySetup = Mesh->GetBodySetup();

            if (MeshIndex < 0 || MeshIndex >= BodySetup->TriMeshGeometries.Num())
                return;

            FString MeshType = "Simple";
            if (ComplexMeshRequired)
                MeshType = "Complex";

            UObject* Outer = BodySetup->GetOuter();
            UStaticMesh* StaticMesh = Cast<UStaticMesh>(Outer);
            FString AssetName = StaticMesh->GetName();
            FString FilePath = FString::Printf(TEXT("%s%s_%s.obj"), *FPaths::ProjectSavedDir(), *MeshType, *AssetName);
            FilePath = FPaths::ConvertRelativePathToFull(FilePath);

            int MeshCount = 0;

            MeshCount = IO::NumFilesExist(FilePath, ComplexMeshRequired);

            if (MeshCount == 0) {

                auto TriMeshData = BodySetup->TriMeshGeometries;
                auto& Vertices = TriMeshData[0].GetReference()->Particles().X();
                UStaticMeshComponent* MeshComponent = Actor->FindComponentByClass<UStaticMeshComponent>();
                FBoxSphereBounds MeshBounds = MeshComponent->CalcBounds(MeshComponent->GetComponentTransform());
                FVector MeshCenter = MeshBounds.Origin;

                if (TriMeshData[0].GetReference()->Elements().RequiresLargeIndices()) {
                    // Use the large index buffer
                    // 32-bit indices
                    const TArray<Chaos::TVector<int32, 3>>& Indices =
                        TriMeshData[0].GetReference()->Elements().GetLargeIndexBuffer(); // 32-bit indices

                    MeshCount = MeshUtils::SaveMesh(FilePath, Vertices, Indices, ComplexMeshRequired);
                } else {
                    // Use the small index buffer
                    // 16-bit indices
                    const TArray<Chaos::TVector<uint16, 3>>& Indices =
                        TriMeshData[0].GetReference()->Elements().GetSmallIndexBuffer(); // 16-bit indices
                                                                                         //
                    MeshCount = MeshUtils::SaveMesh(FilePath, Vertices, Indices, ComplexMeshRequired);
                }
            }

            AMyPhysicsWorldActor::PhysicsObject NewPhysicsObject; // NewPhysicsObject.ObjectName = Actor->GetActorLabel();
            NewPhysicsObject.ObjectName = FString::Printf(
                TEXT("%i"), Comp->GetUniqueID()); // Actor->GetRootComponent()->GetName();//Actor->GetActorNameOrLabel();
            NewPhysicsObject.Actor = Actor;       // Initialize Actor to nullptr or valid actor
            NewPhysicsObject.SceneComponent = Actor->GetRootComponent(); // Initialize Actor to nullptr or valid actor
            NewPhysicsObject.MeshComponent = SMC;
            NewPhysicsObject.Transform = Actor->GetTransform();    // Initialize with default transform
            NewPhysicsObject.Rotation = Actor->GetActorRotation(); // Initialize with default transform
            NewPhysicsObject.Scale = Actor->GetActorScale3D();     // FVector(1.0f, 1.0f, 1.0f);    // Default scale
            NewPhysicsObject.ObjectPathOrPrimitive = FilePath;
            NewPhysicsObject.SubObjectCount = MeshCount;
            NewPhysicsObject.Static = Static; // Set as static for now
            NewPhysicsObject.DrawDebug = true;
            NewPhysicsObject.RobotPart = RobotPart; // Set as static for now
                                                    //

            if (!m_NameToPhysicsObject.Contains(NewPhysicsObject.ObjectName)) {
                m_NameToPhysicsObject.Add(NewPhysicsObject.ObjectName, NewPhysicsObject);
            }

            if (m_mjPhysicsObjects.Contains(NewPhysicsObject.ObjectName)) {
                m_mjPhysicsObjects.Find(NewPhysicsObject.ObjectName)->Add(NewPhysicsObject);

            } else {
                m_mjPhysicsObjects.Add(NewPhysicsObject.ObjectName, TArray<PhysicsObject>{NewPhysicsObject});
            }
        }
    }
}

void AMyPhysicsWorldActor::SetupCameras() {

    // setup cameras
    SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    SceneCaptureComponent->SetupAttachment(RootComponent);
    SceneCaptureComponentDepth = NewObject<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponentDepth"));
    SceneCaptureComponentDepth->SetupAttachment(RootComponent);
    RenderTargetRGB = NewObject<UTextureRenderTarget2D>(this, TEXT("RenderTarget"));
    RenderTargetDepth = NewObject<UTextureRenderTarget2D>(this, TEXT("RenderTargetDepth"));
    RenderTargetRGB->InitAutoFormat(1024, 1024); // Set desired resolution
    SceneCaptureComponent->TextureTarget = RenderTargetRGB;
    SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    SceneCaptureComponent->bCaptureEveryFrame = true;
    SceneCaptureComponent->bCaptureOnMovement = true;
    SceneCaptureComponent->bAlwaysPersistRenderingState = true;
    SceneCaptureComponent->MaxViewDistanceOverride = -1.0f;
    SceneCaptureComponent->PostProcessSettings = SceneCaptureComponent->PostProcessSettings;
    SceneCaptureComponent->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
    SceneCaptureComponent->PostProcessSettings.bOverride_IndirectLightingColor = true;
    SceneCaptureComponent->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
    SceneCaptureComponent->PostProcessSettings.bOverride_ReflectionMethod = true;
    SceneCaptureComponent->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::Lumen;
    SceneCaptureComponent->PostProcessSettings.ReflectionMethod = EReflectionMethod::Lumen;
    SceneCaptureComponent->PostProcessSettings.IndirectLightingIntensity = 1.0f;
    RenderTargetDepth->InitAutoFormat(1024, 1024); // Set desired resolution
    SceneCaptureComponentDepth->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
    SceneCaptureComponentDepth->TextureTarget = RenderTargetDepth;
}

MJHelper::HeightFieldData AMyPhysicsWorldActor::HandleLandscapes() {

    ALandscape* landscape = MeshUtils::FindLandscapeActor(m_World);
    FString OutputPath = FPaths::ProjectSavedDir() / TEXT("HeightField.bin");
    FString OutputPathPng = FPaths::ProjectSavedDir() / TEXT("HeightField.png");
    MJHelper::HeightFieldData heightFieldData =
        MeshUtils::ExtractHeightmapAndSaveToMuJoCo(landscape, OutputPath, OutputPathPng, m_World);
    return heightFieldData;
}

void AMyPhysicsWorldActor::RunMujocoAsync() {

    float IntervalInSeconds = 0.002;
    Async(EAsyncExecution::Thread, [IntervalInSeconds, this]() {
        while (true) // Keep running until manually stopped or object destroyed
        {
            FPlatformProcess::Sleep(IntervalInSeconds);
            if (m_PauseSim)
                continue;

            if (m_JointControlEnabled) {

                for (const TPair<FString, int>& Pair : m_ActuatorAddressesMap) {

                    float* val = m_ActuatorValues.Find(Pair.Key);
                    if (val) {
                        data->ctrl[Pair.Value] = *val;
                    }
                }
            }

            mj_step(model, data);

            if (m_ResetSim) {

                mj_resetData(model, data);
                m_ResetSim = false;
            }
            if (bShouldStopTask) {
                bShouldStopTask = false;
                break;
            }
        }
    });
}
int find_keyframe_index(const mjModel* m, const char* key_name) {
    // Iterate over all keyframe names
    for (int i = 0; i < m->nkey; i++) {
        const char* name = m->names + m->name_keyadr[i];
        if (strcmp(name, key_name) == 0) {
            return i; // Found the keyframe
        }
    }
    return -1; // Not found
}
// Called when the game starts or when spawned
void AMyPhysicsWorldActor::BeginPlay() {
    Super::BeginPlay();

    UWorld* World = GetWorld();
    m_World = World;

    MJHelper::HeightFieldData heightFieldData = HandleLandscapes();

    FName staticTag = FName(TEXT("EP_STATIC"));
    FName staticComplexTag = FName(TEXT("EP_STATIC_COMPLEX"));
    FName dynamicTag = FName(TEXT("EP_DYNAMIC"));
    FName dynamicComplexTag = FName(TEXT("EP_DYNAMIC_COMPLEX"));
    FName robotTag = FName(TEXT("ROBOT"));

    PhysicsStaticActors1 = GetAllActorsWithTag(World, staticTag);
    SetupMJActors(PhysicsStaticActors1, false, false, true);
    PhysicsStaticActors1 = GetAllActorsWithTag(World, staticComplexTag);
    SetupMJActors(PhysicsStaticActors1, false, true, true);
    PhysicsDynamicActors1 = GetAllActorsWithTag(World, robotTag);
    SetupMJActors(PhysicsDynamicActors1, true, false, false);
    PhysicsDynamicActors1 = GetAllActorsWithTag(World, dynamicTag);
    SetupMJActors(PhysicsDynamicActors1, false, false, false);
    PhysicsDynamicActors1 = GetAllActorsWithTag(World, dynamicComplexTag);
    SetupMJActors(PhysicsDynamicActors1, false, true, false);

    XmlManager x;
    char error[1000] = "Could not load binary model";
    model = new mjModel();

    FString fp;
    if (bRecreateMujoco) {

        fp = FString::Printf(TEXT("%shello.xml"), *FPaths::ProjectSavedDir());
        x.UpdateXML(m_mjPhysicsObjects, heightFieldData);
    } else {

        fp = FString::Printf(TEXT("%shello_mod.xml"), *FPaths::ProjectSavedDir());
    }

    fp = FString::Printf(TEXT("%shello_mod.xml"), *FPaths::ProjectSavedDir());
    fp = FPaths::ConvertRelativePathToFull(fp);
    const char* ConvertedString = TCHAR_TO_ANSI(*fp);

    model = mj_loadXML(ConvertedString, 0, error, 1000);
    data = mj_makeData(model);

    mj_step(model, data);
    ComputeMujocoUnrealSetup();
    SetupActuatorAddresses();
    SetupRobotMJtoUE();
    m_RosManager = new RosManager(model, data, &m_ActuatorValues);
    m_RosManager->SetupRos(GetGameInstance());
    // SetupRos("");
    DrawAllBodiesDebug();

    for (int geomId = 0; geomId < model->ngeom; ++geomId) {
        int bodyId = model->geom_bodyid[geomId];

        const char* geomName = mj_id2name(model, mjOBJ_GEOM, geomId);

        FString s = UTF8_TO_TCHAR(geomName);

        int32 SubIndex = s.Find(TEXT("_sub_"), ESearchCase::IgnoreCase, ESearchDir::FromStart);

        // If "_sub_" is found, extract the substring before it
        if (SubIndex != INDEX_NONE) {
            s = s.Left(SubIndex);
        }
        PhysicsObject* p = m_NameToPhysicsObject.Find(s);
        if (p != nullptr && !p->Static) {

            if (!m_MJDynamicToPhysicsObject.Contains(geomId)) {
                m_MJDynamicToPhysicsObject.Add(geomId, p);
            } else {
                m_MJDynamicToPhysicsObject[geomId] = p;
            }
        }
    }
    RunMujocoAsync();
    TArray<AActor*> actors = GetAllActorsWithTag(GetWorld(), "camera");
    if (actors.Num() > 0) {

        UCameraComponent* CameraComponent = actors[0]->FindComponentByClass<UCineCameraComponent>();
        SceneCaptureComponent->SetupAttachment(CameraComponent);
        SceneCaptureComponent->AttachToComponent(actors[0]->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);

        SceneCaptureComponentDepth->SetupAttachment(CameraComponent);
        SceneCaptureComponentDepth->AttachToComponent(actors[0]->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
    }

    TArray<AActor*> GoalActors = GetAllActorsWithTag(GetWorld(), "GoalActor");
    if (GoalActors.Num() > 0) {
        m_GoalActor = GoalActors[0];
    }

    // INFO: Testing of zmq
    // zmq::context_t context;
    // zmq::socket_t socket(context, zmq::socket_type::push);
    // socket.bind("tcp://127.0.0.1:5555");
    // socket.send(zmq::str_buffer("Hello, Unreal!"), zmq::send_flags::none);
}
void AMyPhysicsWorldActor::StopMJ() {
    bShouldStopTask = true;
}

AMyPhysicsWorldActor::MujocoGeomData AMyPhysicsWorldActor::GetMujocoGeomData(int geomId) {
    MujocoGeomData d;

    d.pos = &data->geom_xpos[3 * geomId];
    d.mat = &data->geom_xmat[9 * geomId];

    d.Position = FVector(d.pos[0], -d.pos[1], d.pos[2]);
    d.Position *= Multiplier;
    d.RefPos = FVector(model->geom_pos[3 * geomId], model->geom_pos[3 * geomId + 1], model->geom_pos[3 * geomId + 2]);

    mjtNum _quat[4];
    mju_mat2Quat(_quat, d.mat);
    d.LocalQuatUE = MJHelper::MJQuatToUE(model->geom_quat, 4 * geomId);
    d.GlobalQuatUE = MJHelper::MJQuatToUE(_quat);
    return d;
}

void AMyPhysicsWorldActor::SyncDynamic() {

    Multiplier = 100.0f; // Adjust the multiplier as needed

    for (const TPair<int, PhysicsObject*>& Pair : m_MJDynamicToPhysicsObject) {
        if (Pair.Value == nullptr)
            continue;

        int geomId = Pair.Key;
        int bodyId = model->geom_bodyid[geomId];

        PhysicsObject* p = Pair.Value;

        // if (model->geom_type[geomId] != mjGEOM_MESH)
        //     continue;

        if (model->geom_type[geomId] != mjGEOM_HFIELD) {
            if (p == nullptr)
                continue;
            if (p->Static) // && !p->DrawDebug)
                continue;
        }

        MujocoGeomData geomData = GetMujocoGeomData(geomId);
        mjtNum* pos = geomData.pos;
        mjtNum* mat = geomData.mat;
        FVector Position = geomData.Position;
        FVector refPos = geomData.RefPos;
        FQuat LocalQuatUE = geomData.LocalQuatUE;
        FQuat GlobalQuatUE = geomData.GlobalQuatUE;
        FQuat GlobalQuat = GlobalQuatUE;
        GlobalQuat.X = -GlobalQuat.X;
        GlobalQuat.Z = -GlobalQuat.Z;
        // Quat.Normalize();
        int meshId = model->geom_dataid[geomId];

        if (model->mesh_graphadr[meshId] == -1) {

            continue; // No convex hull data available for this mesh
        }

        FQuat initialQuat = p->Rotation.Quaternion();

        FVector rp = refPos * 100;
        rp.Y *= -1;
        FQuat OffsetQuat = GlobalQuatUE * LocalQuatUE.Inverse();
        FVector OffsetVector = OffsetQuat.RotateVector(p->OffsetPos);
        p->Actor->SetActorRotation(GlobalQuatUE * LocalQuatUE.Inverse());
        p->Actor->SetActorLocation(Position + OffsetVector); // this is the working one for "static" geom
    }
}

void AMyPhysicsWorldActor::DrawAllBodiesDebug() {

    Multiplier = 100.0f; // Adjust the multiplier as needed

    // Iterate through all geoms
    for (int geomId = 0; geomId < model->ngeom; ++geomId) {
        int bodyId = model->geom_bodyid[geomId];

        const char* geomName = mj_id2name(model, mjOBJ_GEOM, geomId);

        FString s = UTF8_TO_TCHAR(geomName);

        int32 SubIndex = s.Find(TEXT("_sub_"), ESearchCase::IgnoreCase, ESearchDir::FromStart);

        // If "_sub_" is found, extract the substring before it
        if (SubIndex != INDEX_NONE) {
            s = s.Left(SubIndex);
        }
        PhysicsObject* p = m_NameToPhysicsObject.Find(s);
        if (model->geom_type[geomId] != mjGEOM_HFIELD) {
            if (p == nullptr)
                continue;
            if (p->Static) // && !p->DrawDebug)
                continue;
        }

        mjtNum* pos; //= &data->xpos[3 * bodyId];
        int id;
        mjtNum* mat; // = &data->xmat[9 * bodyId];
        id = geomId;
        MujocoGeomData geomData = GetMujocoGeomData(id);
        pos = geomData.pos;
        mat = geomData.mat;
        FVector Position = geomData.Position;
        FVector refPos = geomData.RefPos;
        FQuat LocalQuatUE = geomData.LocalQuatUE;
        FQuat GlobalQuatUE = geomData.GlobalQuatUE;
        FQuat GlobalQuat = GlobalQuatUE;
        GlobalQuat.X = -GlobalQuat.X;
        GlobalQuat.Z = -GlobalQuat.Z;
        // Quat.Normalize();
        switch (model->geom_type[geomId]) {
        case mjGEOM_HFIELD: {
            // Get the height field ID and related data
            int hfieldId = model->geom_dataid[geomId];
            float* hfieldData = model->hfield_data + model->hfield_adr[hfieldId];
            int nrow = model->hfield_nrow[hfieldId];
            int ncol = model->hfield_ncol[hfieldId];

            // Dimensions of the height field in the local MuJoCo frame
            float radiusX = model->hfield_size[hfieldId * 4] * Multiplier;
            float radiusY = model->hfield_size[hfieldId * 4 + 1] * Multiplier;
            float elevationZ = model->hfield_size[hfieldId * 4 + 2] * Multiplier;

            // The position of the height field's center in the world
            FVector FieldPosition = Position;
            FieldPosition.Y *= -1;

            // Iterate through height field data and draw points/lines
            for (int i = 0; i < nrow; ++i) {
                for (int j = 0; j < ncol; ++j) {
                    // Compute normalized height [0, 1]
                    float normalizedHeight = hfieldData[i * ncol + j];

                    // Convert to world height
                    float heightZ = normalizedHeight * elevationZ;

                    // Compute the world position of the point
                    FVector PointPosition(FieldPosition.X + (j - ncol / 2.0f) * (2.0f * radiusX / ncol),
                                          FieldPosition.Y - (i - nrow / 2.0f) * (2.0f * radiusY / nrow) + (Position.Y * 2),
                                          FieldPosition.Z + heightZ);

                    // Optionally draw as a grid of points
                    DrawDebugPoint(GetWorld(), PointPosition, 5.0f, FColor::Green, false, -1, 0);
                }
            }
            break;
        }
        case mjGEOM_SPHERE: {
            float radius = model->geom_size[geomId * 3] * Multiplier;
            DrawDebugSphere(GetWorld(), Position, radius, 32, FColor::Green, false, -1, 0, 1.0f);
            break;
        }
        case mjGEOM_CAPSULE: {
            float radius = model->geom_size[geomId * 3] * Multiplier;
            float halfHeight = model->geom_size[geomId * 3 + 1] * Multiplier;
            DrawDebugCapsule(GetWorld(), Position, halfHeight, radius, GlobalQuatUE, FColor::Red, false, -1, 0, 0.3f);
            break;
        }
        case mjGEOM_CYLINDER: {
            float radius = model->geom_size[geomId * 3] * Multiplier;
            float halfHeight = model->geom_size[geomId * 3 + 1] * Multiplier;
            FVector ZAxis = FVector(0, 0, 1);

            // Rotate the z-axis by the quaternion to get the cylinder's orientation
            FVector Axis = GlobalQuatUE.RotateVector(ZAxis);

            // Calculate the start and end points of the cylinder
            FVector Start = Position - Axis * halfHeight;
            FVector End = Position + Axis * halfHeight;
            DrawDebugCylinder(GetWorld(), Start, End, radius, 30, FColor::Red, false, -1, 0, 0.3f);
            break;
        }

        case mjGEOM_BOX: {
            FVector halfExtents(

                model->geom_size[geomId * 3] * Multiplier, model->geom_size[geomId * 3 + 1] * Multiplier,
                model->geom_size[geomId * 3 + 2] * Multiplier);

            // UE_LOG(LogTemp, Warning, TEXT("Drawing box with half extents of %s with am ultiplier of %f"), *halfExtents.ToString(),
            // Multiplier);
            DrawDebugBox(GetWorld(), Position, halfExtents, GlobalQuatUE, FColor::Red, false, -1, 0, 0.3f);
            break;
        }
        case mjGEOM_PLANE: {
            FVector halfExtents(

                model->geom_size[geomId * 3] * Multiplier, model->geom_size[geomId * 3 + 1] * Multiplier, 0.1);
            DrawDebugBox(GetWorld(), Position, halfExtents, GlobalQuatUE, FColor::Blue, false, -1, 0, 0.3f);
            break;
        }
        case mjGEOM_MESH: {
            int meshId = model->geom_dataid[geomId];

            // if (p->Static)
            //     continue;
            // UE_LOG(LogTemp, Warning, TEXT("Trying to debug draw %s"), *s);
            // Check if this mesh has a convex hull
            if (model->mesh_graphadr[meshId] == -1) {

                // UE_LOG(LogTemp, Warning, TEXT("%s has no valid hull"), *s);
                continue; // No convex hull data available for this mesh
            }
            // Access the convex hull data for the mesh
            int graphStart = model->mesh_graphadr[meshId];
            int* graphData = model->mesh_graph + graphStart;

            // Extract the convex hull data
            int numVert = graphData[0];
            int numFace = graphData[1];
            int* vertEdgeAdr = &graphData[2];
            int* vertGlobalId = &graphData[2 + numVert];
            int* edgeLocalId = &graphData[2 + numVert * 2];
            int* faceGlobalId = &edgeLocalId[numVert + 3 * numFace];

            FQuat initialQuat = p->Rotation.Quaternion();
            // FQuat LocalQuatUE = Quat;
            // LocalQuatUE.X = -LocalQuatUE.X;
            // LocalQuatUE.Z = -LocalQuatUE.Z;

            FVector rp = refPos * 100;
            rp.Y *= -1;
            // FQuat OffsetQuat = initialQuat * LocalQuatUE.Inverse();
            // p->Actor->SetActorRotation(p->OffsetQuat * EndQuat); // for static - could have been static?
            // p->Actor->SetActorRelativeLocation(rp + p->OffsetPos); // this is the working one for "static" geom
            //
            FQuat OffsetQuat = GlobalQuatUE * LocalQuatUE.Inverse();
            FVector OffsetVector = OffsetQuat.RotateVector(p->OffsetPos);
            p->Actor->SetActorRotation(GlobalQuatUE * LocalQuatUE.Inverse());
            p->Actor->SetActorLocation(Position + OffsetVector); // this is the working one for "static" geom

            if (true) { // if (m_ShowMujDebug && p->DrawDebug) {
                float* vertices = model->mesh_vert + model->mesh_vertadr[meshId] * 3;
                for (int j = 0; j < numFace; ++j) {
                    // Get the global vertex IDs of the convex hull face
                    int v1_global = faceGlobalId[3 * j];
                    int v2_global = faceGlobalId[3 * j + 1];
                    int v3_global = faceGlobalId[3 * j + 2];

                    // Get the vertices from the full mesh
                    FVector vertex1(vertices[3 * v1_global], vertices[3 * v1_global + 1] * 1, vertices[3 * v1_global + 2] * 1);
                    FVector vertex2(vertices[3 * v2_global], vertices[3 * v2_global + 1] * 1, vertices[3 * v2_global + 2] * 1);
                    FVector vertex3(vertices[3 * v3_global], vertices[3 * v3_global + 1] * 1, vertices[3 * v3_global + 2] * 1);
                    // scale to UE m -> cm
                    vertex1 *= Multiplier;
                    vertex2 *= Multiplier;
                    vertex3 *= Multiplier;

                    vertex1 = GlobalQuat.RotateVector(vertex1);
                    vertex2 = GlobalQuat.RotateVector(vertex2);
                    vertex3 = GlobalQuat.RotateVector(vertex3);
                    // quat is the quat of the geom in muj
                    // vertex1 = Quat.RotateVector(vertex1); // use these for static geom
                    // vertex2 = Quat.RotateVector(vertex2);
                    // vertex3 = Quat.RotateVector(vertex3);
                    FVector tp = Position;
                    tp.Y *= -1;
                    vertex1 += tp;
                    vertex2 += tp;
                    vertex3 += tp;
                    // vertex1 += refPos * 10; // refPos is the position of the geom in muj // use these for static geom
                    // vertex2 += refPos * 10;
                    // vertex3 += refPos * 10;
                    vertex1.Y *= -1;
                    vertex2.Y *= -1;
                    vertex3.Y *= -1;

                    // Draw the edges of the convex hull face
                    DrawDebugLine(GetWorld(), vertex1, vertex2, FColor::Magenta, false, -1, 0, 0.15f);
                    DrawDebugLine(GetWorld(), vertex2, vertex3, FColor::Magenta, false, -1, 0, 0.15f);
                    DrawDebugLine(GetWorld(), vertex3, vertex1, FColor::Magenta, false, -1, 0, 0.15f);
                }
            }

            break;
        }
        default:
            break;
        }
    }
}
void AMyPhysicsWorldActor::SyncMujBodyToUnreal(int body_id, PhysicsObject* p) {
    mjtNum* pos; //= &data->xpos[3 * bodyId];
    mjtNum* mat; // = &data->xmat[9 * bodyId];

    int id = body_id;

    MujocoGeomData geomData = GetMujocoGeomData(id);
    pos = geomData.pos;
    mat = geomData.mat;
    FVector Position = geomData.Position;
    FVector refPos = geomData.RefPos;
    FQuat LocalQuatUE = geomData.LocalQuatUE;
    FQuat GlobalQuatUE = geomData.GlobalQuatUE;
    FQuat initialQuat = p->Rotation.Quaternion();

    FVector rp = refPos * Multiplier;
    rp.Y *= -1;

    FQuat OffsetQuat = GlobalQuatUE * LocalQuatUE.Inverse();
    FVector OffsetVector = OffsetQuat.RotateVector(p->OffsetPos);

    p->MeshComponent->SetWorldRotation(GlobalQuatUE);            // * LocalQuatUE.Inverse());
    p->MeshComponent->SetWorldLocation(Position + OffsetVector); // this is the working one for "static" geom
    int geomId = id;

    switch (model->geom_type[geomId]) {
    case mjGEOM_SPHERE: {
        float radius = model->geom_size[geomId * 3] * Multiplier;
        DrawDebugSphere(GetWorld(), Position, radius, 32, FColor::Green, false, -1, 0, 1.0f);
        break;
    }
    case mjGEOM_CAPSULE: {
        float radius = model->geom_size[geomId * 3] * Multiplier;
        float halfHeight = model->geom_size[geomId * 3 + 1] * Multiplier;
        DrawDebugCapsule(GetWorld(), Position, halfHeight, radius, GlobalQuatUE, FColor::Red, false, -1, 0, 0.3f);
        break;
    }
    case mjGEOM_CYLINDER: {
        float radius = model->geom_size[geomId * 3] * Multiplier;
        float halfHeight = model->geom_size[geomId * 3 + 1] * Multiplier;
        FVector ZAxis = FVector(0, 0, 1);

        // Rotate the z-axis by the quaternion to get the cylinder's orientation
        FVector Axis = GlobalQuatUE.RotateVector(ZAxis);

        // Calculate the start and end points of the cylinder
        FVector Start = Position - Axis * halfHeight;
        FVector End = Position + Axis * halfHeight;
        DrawDebugCylinder(GetWorld(), Start, End, radius, 30, FColor::Red, false, -1, 0, 0.3f);
        break;
    }

    case mjGEOM_BOX: {
        FVector halfExtents(

            model->geom_size[geomId * 3] * Multiplier, model->geom_size[geomId * 3 + 1] * Multiplier,
            model->geom_size[geomId * 3 + 2] * Multiplier);

        // UE_LOG(LogTemp, Warning, TEXT("Drawing box with half extents of %s with am ultiplier of %f"), *halfExtents.ToString(),
        // Multiplier);
        DrawDebugBox(GetWorld(), Position, halfExtents, GlobalQuatUE, FColor::Red, false, -1, 0, 0.3f);
        break;
    }
    case mjGEOM_PLANE: {
        FVector halfExtents(

            model->geom_size[geomId * 3] * Multiplier, model->geom_size[geomId * 3 + 1] * Multiplier, 0.1);
        DrawDebugBox(GetWorld(), Position, halfExtents, GlobalQuatUE, FColor::Blue, false, -1, 0, 0.3f);
        break;
    }
    }
}

void AMyPhysicsWorldActor::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    TArray<FVector> ContactLocations;
    TArray<FVector> ContactForces;

    for (const TPair<int, PhysicsObject*>& Pair : m_MJRobotVisToPhysicsObject) {

        if (Pair.Value != nullptr)
            SyncMujBodyToUnreal(Pair.Key, Pair.Value);
    }

    SyncDynamic();

    if (!m_PublishDataToRos)
        return;
}

void AMyPhysicsWorldActor::BeginDestroy() {
    Super::BeginDestroy();
    // if (m_RosManager != nullptr) {
    //     delete m_RosManager;
    //     m_RosManager = nullptr; // Prevent dangling pointer
    // }
}

void AMyPhysicsWorldActor::SetupActuatorAddresses() {
    // Iterate through all actuators in the model
    for (int i = 0; i < model->nu; i++) { // 'nu' is the number of actuators in the model
        // Get the actuator name
        const char* actuator_name = mj_id2name(model, mjOBJ_ACTUATOR, i);
        int ac_addr = model->actuator_actadr[i];

        // The control values are stored in the 'ctrl' array in mjData
        // Store the actuator name and its control index in the map

        FString MjName(actuator_name);
        UE_LOG(LogTemp, Warning, TEXT("setting up actuator %s to id %i"), *MjName, ac_addr);
        m_ActuatorAddressesMap.Add(FString(actuator_name), i);
        m_ActuatorValues.Add(FString(actuator_name), 0.0f); // Initialize actuator control values to 0
    }
}
void AMyPhysicsWorldActor::UpdateActuatorValuesFromKeyframe( const FString& keyframeName) {
    // Check for a valid model
    if (!model) {
        UE_LOG(LogTemp, Error, TEXT("MuJoCo model is null."));
        return;
    }

    // Find the keyframe index
    int keyframeIndex = find_keyframe_index(model, TCHAR_TO_ANSI(*keyframeName));
    if (keyframeIndex == -1) {
        UE_LOG(LogTemp, Error, TEXT("Keyframe '%s' not found."), *keyframeName);
        return;
    }

    // Get the `ctrl` values from the keyframe
    const mjtNum* ctrl = model->key_ctrl + keyframeIndex * model->nu;
    if (model->nu <= 0) {
        UE_LOG(LogTemp, Warning, TEXT("No actuators in the model."));
        return;
    }

    // Update the map with keyframe `ctrl` values
    for (int i = 0; i < model->nu; i++) {
        FString actuatorName = FString(model->names + model->name_actuatoradr[i]);

        // Check if the actuator exists in the map
        if (m_ActuatorValues.Contains(actuatorName)) {
            // Update the value in the map
            m_ActuatorValues[actuatorName] = static_cast<float>(ctrl[i]);

            // Log the updated value
            UE_LOG(LogTemp, Log, TEXT("Updated Actuator: %s -> Ctrl Value: %f"), *actuatorName, ctrl[i]);
        } else {
            // Log a warning if the actuator is not found in the map
            UE_LOG(LogTemp, Warning, TEXT("Actuator '%s' not found in the map."), *actuatorName);
        }
    }
}
void AMyPhysicsWorldActor::SetupRobotMJtoUE() {

    UWorld* World = GetWorld();

    TArray<AActor*> FoundActors;
    AActor* RobotActorr = nullptr;
    UGameplayStatics::GetAllActorsWithTag(World, FName("ROBOT"), FoundActors);

    int keyframe_index = find_keyframe_index(model, "home");
    UE_LOG(LogTemp, Warning, TEXT("Keyframe id found %i"), keyframe_index)
    const mjtNum* ctrl = model->key_ctrl + keyframe_index * model->nu;
    if (model->nu > 0) {
        for (int i = 0; i < model->nu; i++) {

            UE_LOG(LogTemp, Warning, TEXT("ctrl values for ^ %f"), ctrl[i])
        }
    }
        UpdateActuatorValuesFromKeyframe( "home");

    if (FoundActors.Num() > 0)
        RobotActorr = FoundActors[0];
    if (!RobotActorr)
        return;

    for (int geom_id = 0; geom_id < model->ngeom; geom_id++) {
        // Get the name of the geom
        const char* geom_name = mj_id2name(model, mjOBJ_GEOM, geom_id);
        FString MjName(geom_name);

        if (!MjName.Contains("VIS_"))
            continue;
        for (auto& RobotActor : FoundActors) {

            TInlineComponentArray<UActorComponent*, 30> Components;
            RobotActor->GetComponents(UStaticMeshComponent::StaticClass(), Components);
            // int MeshIndex = 0;
            UCineCameraComponent* CineCameraComp = RobotActor->FindComponentByClass<UCineCameraComponent>();

            if (CineCameraComp) {
                continue;
            }
            for (auto&& Comp : Components) {
                const TArray<FName>& CompTags = Comp->ComponentTags;

                // Log or process the tags
                for (const FName& Tag : CompTags) {

                    if (!MjName.Contains(Tag.ToString()))
                        continue;

                    FString id =
                        FString::Printf(TEXT("%i"),
                                        Comp->GetUniqueID()); // Actor->GetRootComponent()->GetName();//Actor->GetActorNameOrLabel();
                    PhysicsObject* p = m_NameToPhysicsObject.Find(id);

                    if (!m_MJRobotVisToPhysicsObject.Contains(geom_id)) {
                        m_MJRobotVisToPhysicsObject.Add(geom_id, p);
                    } else {
                        m_MJRobotVisToPhysicsObject[geom_id] = p;
                    }
                }
            }
        }
    }
}

// void AMyPhysicsWorldActor::ReadAllSensorDataRos(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg,
//                                                 TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg,
//                                                 TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> TouchForceMsg,
//                                                 FROSTime& ROSTime) {
//     // Check if Model and Data are valid
//     if (!model || !data)
//         return;
//     TouchForceMsg->data.Init(0.0f, 4); // Initialize with 4 elements, all set to 0.0f
//
//     // Loop through all sensors
//     for (int i = 0; i < model->nsensor; i++) {
//         // Get sensor name
//         const char* sensor_name = mj_id2name(model, mjOBJ_SENSOR, i);
//         if (!sensor_name)
//             sensor_name = "Unnamed";
//
//         // Get sensor type
//         int sensor_type = model->sensor_type[i];
//         int sensor_adr = model->sensor_adr[i];
//         int sensor_dim = model->sensor_dim[i];
//
//         // Sort sensor data into relevant categories
//         for (int j = 0; j < sensor_dim; j++) {
//             float sensor_value = data->sensordata[sensor_adr + j];
//
//             if (sensor_type == mjSENS_ACTUATORPOS) {
//                 JointStateMsg->name.Add(sensor_name);      // Add joint name
//                 JointStateMsg->position.Add(sensor_value); // Add joint position
//             } else if (sensor_type == mjSENS_ACTUATORVEL) {
//                 JointStateMsg->velocity.Add(sensor_value); // Add joint velocity
//             } else if (sensor_type == mjSENS_ACTUATORFRC) {
//                 JointStateMsg->effort.Add(sensor_value); // Add joint effort
//             } else if (sensor_type == mjSENS_ACCELEROMETER) {
//
//                 // UE_LOG(LogTemp, Log, TEXT("Should be adding data to imu msg %f"), data->sensordata[sensor_adr]);
//                 ImuMsg->linear_acceleration.x = data->sensordata[sensor_adr];
//                 ImuMsg->linear_acceleration.y = data->sensordata[sensor_adr + 1];
//                 ImuMsg->linear_acceleration.z = data->sensordata[sensor_adr + 2];
//             } else if (sensor_type == mjSENS_GYRO) {
//                 ImuMsg->angular_velocity.x = data->sensordata[sensor_adr];
//                 ImuMsg->angular_velocity.y = data->sensordata[sensor_adr + 1];
//                 ImuMsg->angular_velocity.z = data->sensordata[sensor_adr + 2];
//             } else if (sensor_type == mjSENS_FRAMEQUAT) {
//                 ImuMsg->orientation.x = data->sensordata[sensor_adr];
//                 ImuMsg->orientation.y = data->sensordata[sensor_adr + 1];
//                 ImuMsg->orientation.z = data->sensordata[sensor_adr + 2];
//                 ImuMsg->orientation.w = data->sensordata[sensor_adr + 3];
//
//             } else if (sensor_type == mjSENS_CLOCK) {
//
//                 float mujocoTime = data->sensordata[sensor_adr];
//                 unsigned long seconds = static_cast<unsigned long>(mujocoTime);
//                 unsigned long nanoseconds = static_cast<unsigned long>((mujocoTime - seconds) * 1e9);
//                 ROSTime._Sec = seconds;
//                 ROSTime._NSec = nanoseconds;
//             }
//
//             else if (sensor_type == mjSENS_TOUCH || sensor_type == mjSENS_FORCE) {
//                 // Map specific sensor names to positions in the array
//                 FString SensorNameString(sensor_name);
//                 if (SensorNameString == "contact_sensor_FL") {
//                     TouchForceMsg->data[0] = sensor_value; // Front Left sensor
//                 } else if (SensorNameString == "contact_sensor_FR") {
//                     TouchForceMsg->data[1] = sensor_value; // Front Right sensor
//                 } else if (SensorNameString == "contact_sensor_RL") {
//                     TouchForceMsg->data[2] = sensor_value; // Rear Left sensor
//                 } else if (SensorNameString == "contact_sensor_RR") {
//                     TouchForceMsg->data[3] = sensor_value; // Rear Right sensor
//                 }
//             }
//         }
//     }
// }

// void AMyPhysicsWorldActor::SubCallback_JointState(TSharedPtr<FROSBaseMsg> Msg) {
//     // Cast the message to a JointState message
//     auto CastResponse = StaticCastSharedPtr<ROSMessages::sensor_msgs::JointState>(Msg);
//     if (!CastResponse) {
//         return;
//     }
//
//     // Iterate through the joint names in the message
//     for (int i = 0; i < CastResponse->name.Num(); i++) {
//         // Append "_motor" to the joint name to form the key
//         FString JointNameWithMotor = CastResponse->name[i] + "_motor";
//
//         if (i >= 6)
//             continue;
//
//         // UE_LOG(LogTemp, Warning, TEXT("joint state %s : %f"), *JointNameWithMotor, CastResponse->position[i]);
//         // Check if the key exists in the map
//         if (m_ActuatorValues.Contains(JointNameWithMotor)) {
//
//             m_ActuatorValues[JointNameWithMotor] = CastResponse->position[i];
//         }
//     }
// }
//
// void AMyPhysicsWorldActor::SetupJointStateSub() {
//     m_Topic_JointStateSub = NewObject<UTopic>(UTopic::StaticClass());
//     UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//     m_Topic_JointStateSub->Init(rosinst->ROSIntegrationCore, TEXT("/joint_states"), TEXT("sensor_msgs/JointState"));
//
//     std::function<void(TSharedPtr<FROSBaseMsg>)> callback = [this](TSharedPtr<FROSBaseMsg> msg) {
//         this->SubCallback_JointState(msg);
//     };
//
//     m_Topic_JointStateSub->Subscribe(callback);
// }
//
// void AMyPhysicsWorldActor::PublishJointState(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg, FROSTime RosTime) {
//     // Publish the JointState message
//     if (m_Topic_JointStatePub && ROSInst->bIsConnected) {
//         JointStateMsg->header.time = RosTime;
//         m_Topic_JointStatePub->Publish(JointStateMsg);
//     }
// }
//
// void AMyPhysicsWorldActor::PublishImu(TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg, FROSTime RosTime) {
//     // Publish the Imu message
//     if (m_Topic_ImuPub && ROSInst->bIsConnected) {
//
//         ImuMsg->header.time = RosTime;
//         // UE_LOG(LogTemp, Warning, TEXT("Should be publishign imu %f"), ImuMsg->orientation.x);
//         TArray<double> _temp;
//         _temp.Init(0.0, 9);
//         ImuMsg->orientation_covariance = _temp;
//         ImuMsg->angular_velocity_covariance = _temp;
//         ImuMsg->linear_acceleration_covariance = _temp;
//         m_Topic_ImuPub->Publish(ImuMsg);
//     }
// }
//
// void AMyPhysicsWorldActor::PublishCamera(const TArray<FColor>& OutBMP, uint32 Width, uint32 Height, FROSTime RosTime) {
//
//     if (!m_Topic_CameraPub)
//         return;
//     if (!ROSInst->bIsConnected)
//         return;
//     TSharedPtr<ROSMessages::sensor_msgs::Image> OutRosImage(new ROSMessages::sensor_msgs::Image());
//
//     OutRosImage->header.time = RosTime;
//     // Set image width and height
//     OutRosImage->width = Width;
//     OutRosImage->height = Height;
//
//     // Set encoding to RGB8 (since FColor contains R, G, B, A, we only use R, G, B)
//     OutRosImage->encoding = "rgb8"; // 8-bit RGB
//
//     // Step size: width * number of channels (3 for RGB)
//     OutRosImage->step = Width * 3;
//
//     // is_bigendian: Usually set to 0 for little-endian
//     OutRosImage->is_bigendian = 0;
//
//     // Prepare the data array
//     int32 ImageSize = Width * Height * 3; // Each pixel is 3 bytes (R, G, B)
//     uint8* ImageData = new uint8[ImageSize];
//
//     for (int32 i = 0; i < OutBMP.Num(); i++) {
//         // Copy RGB data from FColor (ignore the Alpha channel)
//         ImageData[i * 3 + 0] = OutBMP[i].R; // Red
//         ImageData[i * 3 + 1] = OutBMP[i].G; // Green
//         ImageData[i * 3 + 2] = OutBMP[i].B; // Blue
//     }
//
//     // Set the pointer to the data (make sure this memory stays valid)
//     OutRosImage->data = ImageData;
//
//     m_Topic_CameraPub->Publish(OutRosImage);
// }
//
// void AMyPhysicsWorldActor::PublishDepth(const TArray<FFloat16Color>& DepthData, uint32 Width, uint32 Height, FROSTime RosTime) {
//
//     TSharedPtr<ROSMessages::sensor_msgs::Image> OutRosImage(new ROSMessages::sensor_msgs::Image());
//     if (!m_Topic_CameraDepthPub)
//         return;
//
//     if (!ROSInst->bIsConnected)
//         return;
//     OutRosImage->header.time = RosTime;
//     // Set image width and height
//     OutRosImage->width = Width;
//     OutRosImage->height = Height;
//
//     // Set encoding to mono8 (grayscale, 8-bit per pixel)
//     OutRosImage->encoding = "mono8"; // 8-bit grayscale
//
//     // Step size: width * 1 (1 byte per pixel for grayscale)
//     OutRosImage->step = Width;
//
//     // is_bigendian: Usually set to 0 for little-endian
//     OutRosImage->is_bigendian = 0;
//
//     // Prepare the data array for grayscale image
//     int32 ImageSize = Width * Height; // Each pixel is 1 byte for grayscale
//     uint8* ImageData = new uint8[ImageSize];
//
//     // Normalize depth values to [0, 255] range for grayscale
//     float MinDepth = FLT_MAX;
//     float MaxDepth = -FLT_MAX;
//
//     // Find min and max depth values to normalize depth data
//     for (int32 i = 0; i < DepthData.Num(); i++) {
//         float DepthValue = DepthData[i].R; // Typically depth is stored in the R channel
//         if (DepthValue < MinDepth)
//             MinDepth = DepthValue;
//         if (DepthValue > MaxDepth)
//             MaxDepth = DepthValue;
//     }
//
//     // Avoid divide-by-zero if all depth values are the same
//     float DepthRange = (MaxDepth - MinDepth) > 0.0f ? (MaxDepth - MinDepth) : 1.0f;
//
//     // Convert depth to 8-bit grayscale
//     for (int32 i = 0; i < DepthData.Num(); i++) {
//         float DepthValue = DepthData[i].R; // Depth is stored in the R channel
//         uint8 GrayscaleValue = static_cast<uint8>(255.0f * (DepthValue - MinDepth) / DepthRange);
//         ImageData[i] = GrayscaleValue;
//     }
//
//     // Set the pointer to the data (make sure this memory stays valid)
//     OutRosImage->data = ImageData;
//
//     m_Topic_CameraDepthPub->Publish(OutRosImage);
// }
//
// void AMyPhysicsWorldActor::PublishContacts(TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> Msg, FROSTime RosTime) {
//     // Publish the JointState message
//     if (m_Topic_ContactsPub && ROSInst->bIsConnected) {
//         m_Topic_ContactsPub->Publish(Msg);
//     }
// }
//
// void AMyPhysicsWorldActor::SubCallback_StartPos(TSharedPtr<FROSBaseMsg> Msg) {
//     // Cast to Pose message
//     auto CastResponse = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(Msg);
//     if (!CastResponse) {
//         return;
//     }
//
//     // Set position from the Pose message
//     m_StartPos.X = CastResponse->position.x;
//     m_StartPos.Y = CastResponse->position.y;
//     m_StartPos.Z = CastResponse->position.z;
//
//     // Set orientation from the Pose message (quaternion)
//     m_Orientation.X = CastResponse->orientation.x;
//     m_Orientation.Y = CastResponse->orientation.y;
//     m_Orientation.Z = CastResponse->orientation.z;
//     m_Orientation.W = CastResponse->orientation.w;
//
//     // Get the body ID of the trunk
//     int body_id = mj_name2id(model, mjOBJ_BODY, "trunk");
//     if (body_id == -1) {
//         // UE_LOG(LogTemp, Warning, TEXT("BODY ID NOT VALID"));
//         return;
//     }
//
//     // Get the joint ID for the free joint associated with the trunk
//     int trunk_joint_id = model->body_jntadr[body_id];
//     if (trunk_joint_id >= 0 && model->jnt_type[trunk_joint_id] == mjJNT_FREE) {
//         // The body has a free joint (6 DOF), so set both the position and orientation
//
//         // Set position (first 3 elements of qpos for the free joint)
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 0] = m_StartPos.X; // Set x
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 1] = m_StartPos.Y; // Set y
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 2] = m_StartPos.Z; // Set z
//
//         // Set orientation (next 4 elements of qpos for quaternion)
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 3] = m_Orientation.W; // w
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 4] = m_Orientation.X; // x
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 5] = m_Orientation.Y; // y
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 6] = m_Orientation.Z; // z
//
//         // UE_LOG(LogTemp, Warning, TEXT("Set position to (%f, %f, %f) and orientation to (%f, %f, %f, %f)"), m_StartPos.X,
//         // m_StartPos.Y,
//         // m_StartPos.Z, m_Orientation.W, m_Orientation.X, m_Orientation.Y, m_Orientation.Z);
//     }
// }
//
// void AMyPhysicsWorldActor::SetupStartPosSub() {
//     m_Topic_StartPosSub = NewObject<UTopic>(UTopic::StaticClass());
//     UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//     m_Topic_StartPosSub->Init(rosinst->ROSIntegrationCore, TEXT("/start_pos"), TEXT("geometry_msgs/Pose"));
//
//     std::function<void(TSharedPtr<FROSBaseMsg>)> callback = [this](TSharedPtr<FROSBaseMsg> msg) {
//         this->SubCallback_StartPos(msg);
//     };
//
//     m_Topic_StartPosSub->Subscribe(callback);
// }
//
// void AMyPhysicsWorldActor::SubCallback_GoalPos(TSharedPtr<FROSBaseMsg> Msg) {
//     // In here we need to apply the torque to the actuators
//
//     auto CastResponse = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(Msg);
//     if (!CastResponse) {
//         return;
//     }
//
//     // Set position from the Pose message
//     FVector GoalPos{CastResponse->position.x * 100, -CastResponse->position.y * 100, CastResponse->position.z * 100};
//
//     // UE_LOG(LogTemp, Warning, TEXT("Setting goalpos to %s"), *GoalPos.ToString());
//     if (m_GoalActor) {
//
//         // UE_LOG(LogTemp, Warning, TEXT("S actor validetting goalpos to %s"), *GoalPos.ToString());
//         m_GoalActor->SetActorLocation(GoalPos);
//     }
// }
// void AMyPhysicsWorldActor::SetupGoalPosSub() {
//     m_Topic_GoalPosSub = NewObject<UTopic>(UTopic::StaticClass());
//     UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//     m_Topic_GoalPosSub->Init(rosinst->ROSIntegrationCore, TEXT("/goal_pos"), TEXT("geometry_msgs/Pose"));
//
//     std::function<void(TSharedPtr<FROSBaseMsg>)> callback = [this](TSharedPtr<FROSBaseMsg> msg) {
//         this->SubCallback_GoalPos(msg);
//     };
//
//     m_Topic_GoalPosSub->Subscribe(callback);
// }
//
// void AMyPhysicsWorldActor::SetupRos(FString Addr) {
//
//     ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//     // ROSInst->ROSBridgeServerHost =  Addr;
//     //
//     UE_LOG(LogTemp, Warning, TEXT("New ros server host is %s"), *ROSInst->ROSBridgeServerHost);
//     // ROSInst->bConnectToROS = true;
//     ROSInst->ConnectToRos();
//
//     if (!ROSInst->bIsConnected) {
//
//         UE_LOG(LogTemp, Warning, TEXT("ROS NOT CONNECTED"));
//         return;
//     }
//
//     UE_LOG(LogTemp, Warning, TEXT("ROS should be CONNECTED"));
//     SetupPublishers();
//     SetupJointStateSub();
//     SetupStartPosSub();
//     SetupGoalPosSub();
// }
//
// void AMyPhysicsWorldActor::SetupPublishers() {
//
//     //
//     //
//     // UE_LOG(LogTemp, Warning, TEXT("ROS INST %i"), ROSInst->bIsConnected);
//
//     if (!ROSInst->bIsConnected)
//         return;
//     m_Topic_ImuPub = NewObject<UTopic>(UTopic::StaticClass());
//
//     m_Topic_ImuPub->Init(ROSInst->ROSIntegrationCore, TEXT("/current_imu"), TEXT("sensor_msgs/Imu"));
//
//     m_Topic_ImuPub->Advertise();
//
//     m_Topic_JointStatePub = NewObject<UTopic>(UTopic::StaticClass());
//     m_Topic_JointStatePub->Init(ROSInst->ROSIntegrationCore, TEXT("/current_joint_state"), TEXT("sensor_msgs/JointState"));
//     m_Topic_JointStatePub->Advertise();
//
//     m_Topic_CameraPub = NewObject<UTopic>(UTopic::StaticClass());
//     m_Topic_CameraPub->Init(ROSInst->ROSIntegrationCore, TEXT("/current_image"), TEXT("sensor_msgs/Image"));
//     m_Topic_CameraPub->Advertise();
//
//     m_Topic_CameraDepthPub = NewObject<UTopic>(UTopic::StaticClass());
//     m_Topic_CameraDepthPub->Init(ROSInst->ROSIntegrationCore, TEXT("/current_depth"), TEXT("sensor_msgs/Image"));
//     m_Topic_CameraDepthPub->Advertise();
//
//     m_Topic_ContactsPub = NewObject<UTopic>(UTopic::StaticClass());
//     m_Topic_ContactsPub->Init(ROSInst->ROSIntegrationCore, TEXT("/current_foot_contacts"), TEXT("std_msgs/Float32MultiArray"));
//     m_Topic_ContactsPub->Advertise();
// }
