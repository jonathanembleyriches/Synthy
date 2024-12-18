#include "Coacd/CoacdInterface.h"
void CoacdInterface::SaveCoACDMeshAsOBJ(CoACD_Mesh& Mesh, const FString& FilePath) {
    FString OutputString;

    // Write vertices
    for (uint64_t i = 0; i < Mesh.vertices_count; ++i) {
        double X = Mesh.vertices_ptr[3 * i];
        double Y = Mesh.vertices_ptr[3 * i + 1];
        double Z = Mesh.vertices_ptr[3 * i + 2];

        // Adjust Unreal Engine coordinates and scale (divide by 100)
        OutputString += FString::Printf(TEXT("v %f %f %f\n"), X / 100.0, -Y / 100.0, Z / 100.0);
    }

    // Write faces (indices are 1-based in OBJ format)
    for (uint64_t i = 0; i < Mesh.triangles_count; ++i) {
        int32 A = Mesh.triangles_ptr[3 * i] + 1;
        int32 B = Mesh.triangles_ptr[3 * i + 1] + 1;
        int32 C = Mesh.triangles_ptr[3 * i + 2] + 1;

        OutputString += FString::Printf(TEXT("f %d %d %d\n"), A, B, C);
    }

    // Save the OBJ string to a file
    if (FFileHelper::SaveStringToFile(OutputString, *FilePath)) {
        UE_LOG(LogTemp, Log, TEXT("Successfully saved mesh to %s"), *FilePath);
    } else {
        UE_LOG(LogTemp, Error, TEXT("Failed to save mesh to %s"), *FilePath);
    }
}

void CoacdInterface::SaveCoACDMeshArrayAsOBJ(CoACD_MeshArray& MeshArray, const FString& BaseFilePath) {
    // for (int32 i = 0; i < MeshArray.meshes_count; ++i) {
    //     FString FilePath = FString::Printf(TEXT("%s_Mesh_%d.obj"), *BaseFilePath, i);
    //     SaveCoACDMeshAsOBJ(MeshArray.meshes_ptr[i], FilePath);
    // }
    // Get the base name of the file without the extension
    FString BaseName = FPaths::GetBaseFilename(BaseFilePath);
    FString Directory = FPaths::GetPath(BaseFilePath);

    for (int32 i = 0; i < MeshArray.meshes_count; ++i) {
        // Construct the new file path with "_sub_n.obj" format
        FString FilePath = FString::Printf(TEXT("%s/%s_sub_%d.obj"), *Directory, *BaseName, i);
        SaveCoACDMeshAsOBJ(MeshArray.meshes_ptr[i], FilePath);
    }
}
