#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include <functional>
namespace IO {

int NumFilesExist(const FString BaseFilePath, const bool ComplexMesh) {

    if (FPaths::FileExists(BaseFilePath))
        return 1;

    if (ComplexMesh) {

        // Convert the relative path to a full path
        FString BaseFileName = FPaths::GetBaseFilename(BaseFilePath);
        // Define a wildcard pattern to match submesh files (e.g., "MeshType_AssetName_sub_*.obj")
        FString WildcardPattern = FString::Printf(TEXT("%s_sub_*.obj"), *BaseFileName);

        // Get the directory of the saved project
        FString Directory = FPaths::GetPath(BaseFilePath);

        // Array to store the found file names
        TArray<FString> FoundFiles;

        // Search for files matching the wildcard pattern
        IFileManager::Get().FindFiles(FoundFiles, *Directory, *WildcardPattern);

        // Count the number of submesh files
        int32 SubMeshCount = FoundFiles.Num();

        UE_LOG(LogTemp, Error, TEXT("Trying to find assets like %s, %i found "), *WildcardPattern, SubMeshCount);
        return SubMeshCount;
    }

    // files not found
    return 0;
}

} // namespace IO
