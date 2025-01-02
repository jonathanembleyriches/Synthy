
#pragma once

#include "Chaos/ArrayCollectionArray.h"

#include "Misc/FileHelper.h"
#include "CoreMinimal.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "LandscapeComponent.h"
#include "Utils/MJHelper.h"
#include "Coacd/CoacdInterface.h"

namespace MeshUtils {

ALandscape* FindLandscapeActor(UWorld* World);
MJHelper::HeightFieldData ExtractHeightmapAndSaveToMuJoCo(ALandscape* LandscapeActor, const FString& OutputFilePath,
                                                          const FString& PNGFilePath, const UWorld* World);
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
