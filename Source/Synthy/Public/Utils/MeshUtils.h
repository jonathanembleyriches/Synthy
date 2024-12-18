
#pragma once

#include "Chaos/ArrayCollectionArray.h"

#include "Misc/FileHelper.h"
#include "CoreMinimal.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "ImageUtils.h"
#include "Kismet/GameplayStatics.h"
#include "Landscape.h"
#include "Landscape.h" // Include this to work with ALandscape
#include "Landscape.h"
#include "LandscapeComponent.h"
#include "LandscapeDataAccess.h"
#include "LandscapeDataAccess.h" // Ensure you include the relevant header
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Utils/MJHelper.h"
#include <fstream>
#include "Coacd/CoacdInterface.h"

namespace MeshUtils {

ALandscape* FindLandscapeActor(UWorld* World);
MJHelper::HeightFieldData ExtractHeightmapAndSaveToMuJoCo(ALandscape* LandscapeActor, const FString& OutputFilePath,
                                                          const FString& PNGFilePath, const UWorld* World);
//
// int SaveMesh(const FString& FilePath, const Chaos::TArrayCollectionArray<Chaos::TVector<float, 3>>& Vertices, const TArray<Chaos::TVector<int, 3>>& Indices, bool ComplexMeshRequired);
//
// int SaveMeshAsOBJSimple(const FString& FilePath, const Chaos::TArrayCollectionArray<Chaos::TVector<float, 3>>& Vertices, const TArray<Chaos::TVector<int, 3>>& Indices);
//
// int SaveMeshAsOBJComplex(const FString& FilePath, const Chaos::TArrayCollectionArray<Chaos::TVector<float, 3>>& Vertices, const TArray<Chaos::TVector<int, 3>>& Indices); 


// template <typename VertexType, typename IndexType>
// int SaveMesh(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
//              const TArray<Chaos::TVector<IndexType, 3>>& Indices, bool ComplexMeshRequired);
//
// template <typename VertexType, typename IndexType>
// int SaveMeshAsOBJSimple(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
//                         const TArray<Chaos::TVector<IndexType, 3>>& Indices);
//
// template <typename VertexType, typename IndexType>
// int SaveMeshAsOBJComplex(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
//                          const TArray<Chaos::TVector<IndexType, 3>>& Indices);
template <typename VertexType, typename IndexType>
int SaveMeshAsOBJSimple(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
                        const TArray<Chaos::TVector<IndexType, 3>>& Indices) {
    FString OutputString;

    // Write vertices
    for (const auto& Vertex : Vertices) {
        OutputString += FString::Printf(TEXT("v %f %f %f\n"), Vertex.X / 100, -Vertex.Y / 100, Vertex.Z / 100);
    }

    // Write indices
    for (const auto& Index : Indices) {
        OutputString += FString::Printf(TEXT("f %d %d %d\n"), Index.X + 1, Index.Y + 1, Index.Z + 1);
    }

    return FFileHelper::SaveStringToFile(OutputString, *FilePath) ? 1 : 0;
}

template <typename VertexType, typename IndexType>
int SaveMeshAsOBJComplex(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
                         const TArray<Chaos::TVector<IndexType, 3>>& Indices) {
    CoACD_Mesh inputMesh = CoacdInterface::ConvertToCoACDMesh(Vertices, Indices);

    CoACD_MeshArray result = CoACD_run(inputMesh, 0.05, -1, preprocess_auto, 50, 2000, 20, 150, 3, false, true, false, 100,
                                       false, 0.01, apx_ch, 0);
    int MeshCount = result.meshes_count;

    UE_LOG(LogTemp, Warning, TEXT("Number of meshes created: %i"), result.meshes_count);

    CoacdInterface::SaveCoACDMeshArrayAsOBJ(result, FilePath);
    CoACD_freeMeshArray(result);
    return MeshCount;
}

template <typename VertexType, typename IndexType>
int SaveMesh(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
             const TArray<Chaos::TVector<IndexType, 3>>& Indices, bool ComplexMeshRequired) {
    if (ComplexMeshRequired)
        return MeshUtils::SaveMeshAsOBJComplex(FilePath, Vertices, Indices);

    return MeshUtils::SaveMeshAsOBJSimple(FilePath, Vertices, Indices);
}

}
