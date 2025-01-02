#include "Utils/MeshUtils.h"
#include "Utils/SynthyLogging.h"
ALandscape* MeshUtils::FindLandscapeActor(UWorld* World) {
    if (!World) {
        UE_LOG(SynthyLog, Warning, TEXT("World is null!"));
        return nullptr;
    }

    // Iterate over all actors in the world
    for (TActorIterator<ALandscape> It(World); It; ++It) {
        ALandscape* Landscape = *It;
        if (Landscape) {
            UE_LOG(SynthyLog, Log, TEXT("Found a landscape actor: %s"), *Landscape->GetName());
            // Perform operations with the landscape actor
            return Landscape;
        }
    }

    return nullptr;
}
MJHelper::HeightFieldData MeshUtils::ExtractHeightmapAndSaveToMuJoCo(ALandscape* LandscapeActor, const FString& OutputFilePath, const FString& PNGFilePath,
                                                const UWorld* World) {
    MJHelper::HeightFieldData heightFieldData;
    if (!LandscapeActor) {
        UE_LOG(SynthyLog, Warning, TEXT("Landscape actor is null!"));
        return heightFieldData;
    }

    // Get the landscape scale
    FVector LandscapeScale = LandscapeActor->GetActorScale3D();
    float HeightRange = 512.0f * LandscapeScale.Z; // Unreal's default range is ±256m for a Z-scale of 1.

    TArray<float> HeightData;
    int32 TotalWidth = 0;
    int32 TotalHeight = 0;
    bool complete = false;
    int32 count = 0;
    FVector OutSize;
    FBox OutBounds;
    int32 TotalComponentsX = 0;
    int32 TotalComponentsY = 0;
    FBox Bounds(ForceInit);
    for (ULandscapeComponent* Component : LandscapeActor->LandscapeComponents) {
        if (!Component)
            continue;

        // Get the section base for this component
        FIntPoint SectionBase = Component->GetSectionBase();

        // Calculate the component size in world space
        int32 ComponentSizeQuads = Component->ComponentSizeQuads;
        FVector ComponentSize = FVector(ComponentSizeQuads, ComponentSizeQuads, 0) * LandscapeActor->GetActorScale3D();

        // Expand the bounds to include this component
        FVector MinWorld = FVector(SectionBase.X, SectionBase.Y, 0) * LandscapeActor->GetActorScale3D();
        FVector MaxWorld = MinWorld + ComponentSize;

        Bounds += FBox(MinWorld, MaxWorld);

        // Track the total components in X and Y directions
        TotalComponentsX = FMath::Max(TotalComponentsX, SectionBase.X + ComponentSizeQuads);
        TotalComponentsY = FMath::Max(TotalComponentsY, SectionBase.Y + ComponentSizeQuads);
    }

    // Output the overall size and bounds
    OutSize = FVector(TotalComponentsX, TotalComponentsY, 0) * LandscapeActor->GetActorScale3D();
    OutBounds = Bounds;

    for (ULandscapeComponent* Component : LandscapeActor->LandscapeComponents) {
        if (!Component) {
            continue;
        }

        UTexture2D* HeightmapTexture = Component->GetHeightmap(false);
        UTexture2D* OffsetMapTexture = Component->XYOffsetmapTexture;
        if (!HeightmapTexture || !HeightmapTexture->Source.IsValid()) {
            UE_LOG(SynthyLog, Warning, TEXT("No valid heightmap texture or source for component: %s"), *Component->GetName());
            continue;
        }

        FTextureSource& HeightmapSource = HeightmapTexture->Source;
        int32 Width = HeightmapSource.GetSizeX();
        int32 Height = HeightmapSource.GetSizeY();

        if (Width == 0 || Height == 0) {
            UE_LOG(SynthyLog, Warning, TEXT("Heightmap texture source has invalid dimensions for component: %s"), *Component->GetName());
            continue;
        }

        // UE_LOG(LogTemp, Warning, TEXT("Total height %i Total widht %i"), Height, Width);
        if (complete)
            continue;
        if (TotalWidth == 0) {
            TotalWidth = Width;
            TotalHeight = Height;
        }

        // const uint8* HeightmapData = HeightmapSource.LockMip(0);

        FColor* hmap_data = (FColor*)HeightmapSource.LockMip(0);
        if (!hmap_data) {
            UE_LOG(SynthyLog, Warning, TEXT("Failed to lock heightmap data for component: %s"), *Component->GetName());
            continue;
        }

        float max = 0;

        heightFieldData.mjXSize = (OutSize.X / 100.0f / 2);
        heightFieldData.mjYSize = (OutSize.Y / 100.0f / 2);

        heightFieldData.mjHeight = HeightRange / 100;
        heightFieldData.mjXPos = heightFieldData.mjXSize + (LandscapeActor->GetActorLocation().X/100);
        heightFieldData.mjYPos = -heightFieldData.mjYSize - (LandscapeActor->GetActorLocation().Y/100);
        heightFieldData.mjZPos = -(heightFieldData.mjHeight / 2) + (LandscapeActor->GetActorLocation().Z/100);
        heightFieldData.valid=true;

        // UE_LOG(SynthyLog, Log, TEXT("size = %i %i %i %i pos = %i %i %i"), mjXSize, mjYSize, mjHeight, mjHeight, mjXPos, mjYPos, mjZPos);

        // wasA 5034000
        float ScaleOff = (OutSize.X) / (Height - 1);

        // for (int32 Y = 0; Y < Height; Y++) {
        for (int32 Y = TotalHeight - 1; Y >= 0; Y--) { // Iterate from the last row to the first row
            for (int32 X = 0; X < Width; X++) {
                int32 Index = (Y * Width + X) * 4; // RGBA = 4 bytes per pixel
                // uint8 R = HeightmapData[Index];
                // uint8 G = HeightmapData[Index + 1];
                // uint16 EncodedHeight = (R << 8) + G;
                // float LocalHeight = LandscapeDataAccess::GetLocalHeight(EncodedHeight);

                FColor height = hmap_data[Y * Width + X];
                uint16 LocalHeight = (height.R << 8) | height.G;
                float sc = (1.0f / 256.f);

                // float HeightRange = 256.0f;
                float NormalizedHeight = ((float)LocalHeight / 65535.0f) * (512 * 100) + -(256 * 100); // - (HeightRange / 2.0f
                NormalizedHeight = ((float)LocalHeight / (65535.0f));
                // NormalizedHeight = NormalizedHeight * 512.0f;
                float Elevation = ((float)LocalHeight / 65535.0f) * 512.0f - 256.0f;
                if (Elevation > max)
                    max = Elevation;
                float ScaleFactor = LandscapeScale.X; // Horizontal scale factor
                FVector ComponentLocation = Component->GetComponentLocation();
                // FVector PointPosition = FVector(X*100  , Y*100  , Elevation*100);
                FVector PointPosition = FVector(X * 50400 / (Width - 1), Y * 50400 / (Height - 1), Elevation * 100);

                // Draw debug point
                // DrawDebugPoint(World, PointPosition, 10.0f, FColor::Red, true, 9999, 0);
                // if(X > 500 && Y > 500)

                // UE_LOG(SynthyLog, Log, TEXT("X: %i, Y: %i, %s"),X,Y,*PointPosition.ToString());
                // if(count < 1000000)
                //     DrawDebugPoint(World, FVector(X, Y, Elevation) * 100.0f, 5.0f, FColor::Red, true, 9999, 0);

                count += 1;
                // Normalize Elevation to [0, 1] based on the given height field rules
                NormalizedHeight = (Elevation + 256.0f) / 512.0f;

                // NormalizedHeight = NormalizedHeight * HeightRange - (HeightRange / 2.0f);
                // UE_LOG(LogTemp, Log, TEXT("%f"),NormalizedHeight);
                // Normalize to [0, 1]
                // float NormalizedHeight = (LocalHeight + 256.0f) / 512.0f;
                if (!complete)
                    HeightData.Add(NormalizedHeight);
                // HeightData.Add(Elevation);
            }
        }

        HeightData[0] = 0.0f;
        HeightData[1] = 1.0f;
        complete = true;
        HeightmapSource.UnlockMip(0);
    }

    // // Save Heightmap Data to a PNG
    // if (HeightData.Num() > 0 && TotalWidth > 0 && TotalHeight > 0) {
    //     TArray<FColor> ColorData;
    //     ColorData.Reserve(HeightData.Num());
    //
    //     // Convert normalized height data to grayscale FColor
    //     for (float NormalizedHeight : HeightData) {
    //         uint8 GrayValue = FMath::Clamp(NormalizedHeight * 255.0f, 0.0f, 255.0f);
    //         ColorData.Add(FColor(GrayValue, GrayValue, GrayValue, 255));
    //     }
    //
    //     // Create PNG
    //     TArray<uint8> CompressedPNG;
    //     FImageUtils::CompressImageArray(TotalWidth, TotalHeight, ColorData, CompressedPNG);
    //
    //     // Save to disk
    //     FFileHelper::SaveArrayToFile(CompressedPNG, *PNGFilePath);
    //     UE_LOG(LogTemp, Log, TEXT("Heightmap PNG saved to: %s"), *PNGFilePath);
    // }

    std::ofstream OutputFile(TCHAR_TO_UTF8(*OutputFilePath), std::ios::binary);
    if (!OutputFile) {
        UE_LOG(SynthyLog, Error, TEXT("Failed to open output file: %s"), *OutputFilePath);
        return heightFieldData;
    }

    // Write the header
    int32 RowCount = TotalHeight;
    int32 ColCount = TotalWidth;
    OutputFile.write(reinterpret_cast<const char*>(&RowCount), sizeof(int32));
    OutputFile.write(reinterpret_cast<const char*>(&ColCount), sizeof(int32));

    // Write the height data
    for (float _Height : HeightData) {
        OutputFile.write(reinterpret_cast<const char*>(&_Height), sizeof(float));
    }

    OutputFile.close();
    return heightFieldData;
}



