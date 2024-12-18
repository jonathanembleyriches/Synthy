#pragma once
#include "Mujoco/MyPhysicsWorldActor.h"
#include "XmlFile.h"
// #include "Utils/MeshUtils.h"
#include "Utils/MJHelper.h"

class XmlManager {
public:
    XmlManager();

    void LoadXML();

    void AddGeom();

    void UpdateXML(TMap<FString, TArray<AMyPhysicsWorldActor::PhysicsObject>> PhysicsObjects,
                   MJHelper::HeightFieldData HeightFieldData);
    // void AddStaticGeometry(const FString& Type, const FVector& Size, const FLinearColor& Color);
    // void AddDynamicGeometry(const FString& Type, const FVector& Size, const FLinearColor& Color, const FVector& Position);
    FString GetFileAsString();

private:
    FXmlFile m_File;
    FString FilePath;
    FXmlFile XmlFile;
    FXmlNode* WorldBodyNode;

    FXmlNode* FindWorldBodyNode();

    void CreateFreeNode();

    void AddNode(FString tag, FString Name, FString Type, FVector Size, FVector Pos, FVector Euler) {

        WorldBodyNode->AppendChildNode(tag, "", CreateAttributes(Name, Type, Size, Pos, Euler));
    }

    void AddStaticGeometry(FString tag, FString Name, FString Type, FVector Size, FVector Pos, FVector Euler) {

        WorldBodyNode->AppendChildNode(tag, "", CreateAttributes(Name, Type, Size, Pos, Euler));
    }

    void AddGeometry(FString tag, FString Name, FString Type, FVector Size, FVector Pos, FVector Euler) {

        WorldBodyNode->AppendChildNode(tag, "", CreateAttributes(Name, Type, Size, Pos, Euler));

        // Add the body
        // WorldBodyNode->AppendChildNode(Name, "", nullptr);

        // now we also
    }
    TArray<FXmlAttribute> CreateAttributes(FString Name, FString Type, FVector Size, FVector Pos, FVector Euler) {
        FString SizeString = FString::Printf(TEXT("%.0f %.0f %.0f"), Size.X, Size.Y, Size.Z);
        FString PosString = FString::Printf(TEXT("%.0f %.0f %.0f"), Pos.X, Pos.Y, Pos.Z);
        FString EulerString = FString::Printf(TEXT("%.0f %.0f %.0f"), Euler.X, Euler.Y, Euler.Z);

        TArray<FXmlAttribute> atts;

        FXmlAttribute name("name", Name);
        FXmlAttribute type("type", Type);
        FXmlAttribute size("size", SizeString);
        FXmlAttribute pos("pos", PosString);
        FXmlAttribute euler("euler", EulerString);
        atts.Add(name);
        atts.Add(type);
        atts.Add(size);
        atts.Add(pos);
        atts.Add(euler);
        return atts;
    }

    FXmlNode* FindNode(FString Name) {
        if (FXmlNode* MujocoNode = m_File.GetRootNode()) {
            if (MujocoNode) {
                for (auto child : MujocoNode->GetChildrenNodes()) {
                    if (child->GetTag().Equals(Name))
                        return child;
                }
            }
        }
        return nullptr;
    }

    void AddHeightFieldAsset(FString AssetName, FString AssetFilePath, MJHelper::HeightFieldData HeightFieldData) {

        FXmlNode* AssetNode = FindNode("asset");

        if (AssetNode == nullptr) {

            // UE_LOG(LogTemp, Error, TEXT(" THE ASSET NODE IS A NULLPTR"));
            return;
        }

        // UE_LOG(LogTemp, Error, TEXT("Asset node FOUND creating with %s %s "), *AssetName, *AssetFilePath);

        // TODO: Here we need to check if it already exists, we also want to pre_index it
        //
        //
        FString sizeString = FString::Printf(TEXT("%i %i %i %i"), HeightFieldData.mjXSize, HeightFieldData.mjYSize,
                                             HeightFieldData.mjHeight, HeightFieldData.mjHeight);
        TArray<FXmlAttribute> atts;

        FString OutputPath = FPaths::ProjectSavedDir() / TEXT("HeightField.bin");

    OutputPath = FPaths::ConvertRelativePathToFull(OutputPath);
                    // OutputPath= FPaths::GetPath(OutputPath);
        FXmlAttribute name("name", "terrain");
        FXmlAttribute file("file",OutputPath );
        FXmlAttribute scale("size", sizeString);
        atts.Add(name);
        atts.Add(file);
        atts.Add(scale);

        AssetNode->AppendChildNode("hfield", "", atts);
    }
    void AddMeshAsset(FString AssetName, FString AssetFilePath, FVector Scale) {

        FXmlNode* AssetNode = FindNode("asset");

        if (AssetNode == nullptr) {

            // UE_LOG(LogTemp, Error, TEXT(" THE ASSET NODE IS A NULLPTR"));
            return;
        }

        // UE_LOG(LogTemp, Error, TEXT("Asset node FOUND creating with %s %s "), *AssetName, *AssetFilePath);

        // TODO: Here we need to check if it already exists, we also want to pre_index it
        //
        //
        FString ScaleString = FString::Printf(TEXT("%.4f %.4f %.4f"), Scale.X, Scale.Y, Scale.Z);
        auto ChildNodes = AssetNode->GetChildrenNodes();
        for (FXmlNode* n : ChildNodes) {
            if (n->GetAttribute("name").Equals("ASSET_" + AssetName)) {
                return;
            }
        }
        TArray<FXmlAttribute> atts;
        FXmlAttribute name("name", "ASSET_" + AssetName);
        FXmlAttribute file("file", AssetFilePath);
        FXmlAttribute scale("scale", ScaleString);
        atts.Add(name);
        atts.Add(file);
        atts.Add(scale);

        AssetNode->AppendChildNode("mesh", "", atts);
    }

    TArray<FXmlAttribute> CreateBodyAttributesQuat(FString Name, FVector Pos, FQuat Quat, FVector Scale) {
        FString PosString = FString::Printf(TEXT("%.4f %.4f %.4f"), Pos.X, Pos.Y, Pos.Z);
        FString EulerString = FString::Printf(TEXT("%.5f %.5f %.5f %.5f"), Quat.W, Quat.X, Quat.Y, Quat.Z);

        FString ScaleString = FString::Printf(TEXT("%.4f %.4f %.4f"), Scale.X, Scale.Y, Scale.Z);
        TArray<FXmlAttribute> atts;

        FXmlAttribute name("name", Name);
        FXmlAttribute pos("pos", PosString);
        FXmlAttribute euler("quat", EulerString);

        // FXmlAttribute scale("scale",ScaleString);
        atts.Add(name);
        atts.Add(pos);
        atts.Add(euler);
        // atts.Add(scale);
        return atts;
    }

    TArray<FXmlAttribute> CreateMeshAttributesSimpleWithQuat(FString Name, FString Type, FString mesh_type, FQuat Quat) {
        TArray<FXmlAttribute> atts;

        FString EulerString = FString::Printf(TEXT("%.5f %.5f %.5f %.5f"), Quat.W, Quat.X, Quat.Y, Quat.Z);
        FXmlAttribute name("name", Name);
        FXmlAttribute type("type", Type);
        FXmlAttribute mtype("mesh", mesh_type);

        FXmlAttribute euler("quat", EulerString);
        atts.Add(name);
        atts.Add(type);
        atts.Add(mtype);

        atts.Add(euler);
        return atts;
    }
    TArray<FXmlAttribute> CreateMeshAttributesSimple(FString Name, FString Type, FString mesh_type) {
        TArray<FXmlAttribute> atts;

        FXmlAttribute name("name", Name);
        FXmlAttribute type("type", Type);
        FXmlAttribute mtype("mesh", mesh_type);
        atts.Add(name);
        atts.Add(type);
        atts.Add(mtype);
        return atts;
    }
    TArray<FXmlAttribute> CreateMeshAttributesQuat(FString Name, FString Type, FString mesh_type, FVector Pos, FQuat Quat) {
        FString PosString = FString::Printf(TEXT("%.4f %.4f %.4f"), Pos.X, Pos.Y, Pos.Z);
        FString EulerString = FString::Printf(TEXT("%.5f %.5f %.5f %.5f"), Quat.W, Quat.X, Quat.Y, Quat.Z);

        TArray<FXmlAttribute> atts;

        FXmlAttribute name("name", Name);
        FXmlAttribute type("type", Type);
        FXmlAttribute mtype("mesh", mesh_type);
        FXmlAttribute pos("pos", PosString);
        FXmlAttribute euler("quat", EulerString);

        atts.Add(name);
        atts.Add(type);
        atts.Add(mtype);
        atts.Add(pos);
        atts.Add(euler);

        return atts;
    }
    TArray<FXmlAttribute> CreateMeshAttributes(FString Name, FString Type, FString mesh_type, FVector Pos, FVector Euler) {
        FString PosString = FString::Printf(TEXT("%.4f %.4f %.4f"), Pos.X, Pos.Y, Pos.Z);
        FString EulerString = FString::Printf(TEXT("%.5f %.5f %.5f"), Euler.X, Euler.Y, Euler.Z);
        TArray<FXmlAttribute> atts;

        FXmlAttribute name("name", Name);
        FXmlAttribute type("type", Type);
        FXmlAttribute mtype("mesh", mesh_type);
        FXmlAttribute pos("pos", PosString);
        FXmlAttribute euler("euler", EulerString);
        atts.Add(name);
        atts.Add(type);
        atts.Add(mtype);
        atts.Add(pos);
        atts.Add(euler);
        return atts;
    }
};
