#pragma once
#include "coacd.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include <functional>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include <functional>
#include <mujoco/mujoco.h>
#include "Chaos/ArrayCollectionArray.h"

#include "Misc/FileHelper.h"
namespace CoacdInterface {

template <typename VertexType, typename IndexType>
CoACD_Mesh ConvertToCoACDMesh(const Chaos::TArrayCollectionArray<VertexType>& Vertices, const TArray<Chaos::TVector<IndexType, 3>>& Indices) {

    CoACD_Mesh coacdMesh;

    // Allocate memory for vertices
    coacdMesh.vertices_count = Vertices.Num();
    coacdMesh.vertices_ptr = new double[coacdMesh.vertices_count * 3];

    // Copy vertex data (FVector to double array)
    for (int i = 0; i < Vertices.Num(); ++i) {
        coacdMesh.vertices_ptr[3 * i] = Vertices[i].X;
        coacdMesh.vertices_ptr[3 * i + 1] = Vertices[i].Y;
        coacdMesh.vertices_ptr[3 * i + 2] = Vertices[i].Z;
    }

    // Allocate memory for indices
    coacdMesh.triangles_count = Indices.Num();
    coacdMesh.triangles_ptr = new int[coacdMesh.triangles_count * 3];

    // Copy index data (int32 array)
    for (int i = 0; i < Indices.Num(); ++i) {
        coacdMesh.triangles_ptr[3 * i] = Indices[i][0];
        coacdMesh.triangles_ptr[3 * i + 1] = Indices[i][1];
        coacdMesh.triangles_ptr[3 * i + 2] = Indices[i][2];
    }

    return coacdMesh;
}
void SaveCoACDMeshAsOBJ(CoACD_Mesh& Mesh, const FString& FilePath) ;

void SaveCoACDMeshArrayAsOBJ(CoACD_MeshArray& MeshArray, const FString& BaseFilePath) ;
}
