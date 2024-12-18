#include "Utils/XmlManager.h"
#include "XmlParser.h"

XmlManager::XmlManager() {}
void LogXmlNodeRecursive(const FXmlNode* Node, int32 IndentLevel = 0) {
    if (!Node)
        return;

    // Create indentation for better readability
    FString Indentation = FString(TEXT("")).RightPad(IndentLevel * 4);

    // Log the tag name
    UE_LOG(LogTemp, Log, TEXT("%s<Tag>: %s"), *Indentation, *Node->GetTag());

    // Log all attributes without any special handling
    for (const auto& Attribute : Node->GetAttributes()) {
        UE_LOG(LogTemp, Log, TEXT("%s  [Attribute] %s = %s"), *Indentation, *Attribute.GetTag(), *Attribute.GetValue());
    }

    // Log content if present
    FString Content = Node->GetContent();
    if (!Content.IsEmpty()) {
        UE_LOG(LogTemp, Log, TEXT("%s  [Content]: %s"), *Indentation, *Content);
    }

    // Recursively log all child nodes
    for (const FXmlNode* ChildNode : Node->GetChildrenNodes()) {
        LogXmlNodeRecursive(ChildNode, IndentLevel + 1);
    }
}
/*
void LogXmlNodeRecursive(const FXmlNode* Node, int32 IndentLevel = 0)
{
        if (!Node) return;

        // Create indentation for better readability
        FString Indentation = FString(TEXT("")).RightPad(IndentLevel * 4);

        // Log the tag name
        UE_LOG(LogTemp, Log, TEXT("%s<Tag>: %s"), *Indentation, *Node->GetTag());


        // Log attributes
        for (const auto& Attribute : Node->GetAttributes())
        {
                UE_LOG(LogTemp, Log, TEXT("%s  [Attribute] %s = %s"), *Indentation, *Attribute.GetTag(), *Attribute.GetValue());
        }

        // Log content if present
        FString Content = Node->GetContent();
        if (!Content.IsEmpty())
        {
                UE_LOG(LogTemp, Log, TEXT("%s  [Content]: %s"), *Indentation, *Content);
        }

        // Recursively log all child nodes
        for (const FXmlNode* ChildNode : Node->GetChildrenNodes())
        {
                LogXmlNodeRecursive(ChildNode, IndentLevel + 1);
        }
}*/

void LogXmlFileData(const FString& FilePath) {
    FString XmlContent;

    if (FFileHelper::LoadFileToString(XmlContent, *FilePath)) {
        FXmlFile XmlFile(XmlContent, EConstructMethod::ConstructFromBuffer);

        if (XmlFile.IsValid()) {
            UE_LOG(LogTemp, Log, TEXT("Successfully loaded XML file: %s"), *FilePath);
            const FXmlNode* RootNode = XmlFile.GetRootNode();
            LogXmlNodeRecursive(RootNode);
        } else {
            UE_LOG(LogTemp, Error, TEXT("Failed to parse XML file: %s"), *FilePath);
        }
    } else {
        UE_LOG(LogTemp, Error, TEXT("Failed to load XML file: %s"), *FilePath);
    }
}
void XmlManager::LoadXML() {

    FString fp; // = TEXT("C:\\Users\\jonem\\Downloads\\hello.xml");

    // fp = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Content/XMLDATA/hello.xml"));
    //
    //     fp= FPaths::Combine(FPaths::ProjectContentDir(), TEXT("XMLDATA/hello.xml"));

    fp = FString::Printf(TEXT("%shello.xml"), *FPaths::ProjectSavedDir());

    fp = FPaths::ConvertRelativePathToFull(fp);
    m_File.LoadFile(fp);
    FString XmlContent;

    if (FFileHelper::LoadFileToString(XmlContent, *fp)) {
        UE_LOG(LogTemp, Error, TEXT("%s"), *XmlContent);
    } else {
        UE_LOG(LogTemp, Error, TEXT("Failed to load XML file."));
    }
    LogXmlFileData(fp);
    WorldBodyNode = FindWorldBodyNode();
    FVector x(1.0, 1.0, 1.0);
    AddNode("geom", "eg1", "box", x, x, x);

    //     FString fp_save;// = TEXT("C:\\Users\\jonem\\Downloads\\hello_mod.xml");
    //
    // // fp_save = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("Content/XMLDATA/hello_mod.xml"));
    //
    //     fp_save= FPaths::Combine(FPaths::ProjectContentDir(), TEXT("XMLDATA/hello_mod.xml"));
    //
    //     fp= FString::Printf(TEXT("%shello_mod.xml"), *FPaths::ProjectSavedDir());
    //
    //     fp_save= FPaths::ConvertRelativePathToFull(fp);
    //     m_File.Save(fp_save);
}

void XmlManager::UpdateXML(TMap<FString, TArray<AMyPhysicsWorldActor::PhysicsObject>> PhysicsObjects,
                           MJHelper::HeightFieldData HeightFieldData) {

    FString fp; // = TEXT("C:\\Users\\jonem\\Downloads\\hello.xml");

    // fp= FPaths::Combine(FPaths::ProjectContentDir(), TEXT("XMLDATA/hello.xml"));
    fp = FString::Printf(TEXT("%shello.xml"), *FPaths::ProjectSavedDir());
    fp = FPaths::ConvertRelativePathToFull(fp);
    m_File.LoadFile(fp);
    FString XmlContent;

    // if (FFileHelper::LoadFileToString(XmlContent, *fp)) {
    //     UE_LOG(LogTemp, Error, TEXT("%s"), *XmlContent);
    // } else {
    //     UE_LOG(LogTemp, Error, TEXT("Failed to load XML file."));
    // // }
    // LogXmlFileData(fp);
    WorldBodyNode = FindWorldBodyNode();

    if (HeightFieldData.valid){
        AddHeightFieldAsset("", "", HeightFieldData);

        TArray<FXmlAttribute> atts;
        FString PosString = FString::Printf(TEXT("%i %i %i"),HeightFieldData.mjXPos,HeightFieldData.mjYPos,HeightFieldData.mjZPos);
        FXmlAttribute type("type", "hfield");
        FXmlAttribute name("name", "terrain_geom");
        FXmlAttribute pos("pos",PosString);
        FXmlAttribute hfield("hfield","terrain");
        atts.Add(type);
        atts.Add(name);
        atts.Add(pos);
        atts.Add(hfield);

        WorldBodyNode->AppendChildNode("geom", "", atts);
    }
    for (const TPair<FString, TArray<AMyPhysicsWorldActor::PhysicsObject>>& Pair : PhysicsObjects) {

        // Log the key (AssetName)
        FString AssetName = Pair.Key;
        // UE_LOG(LogTemp, Warning, TEXT("Asset Name: %s"), *AssetName);
        TArray<FString> CreatedAssets;

        // Iterate through the TArray of PhysicsObjects
        const TArray<AMyPhysicsWorldActor::PhysicsObject>& PhysicsObjectsArray = Pair.Value;
        for (int32 i = 0; i < PhysicsObjectsArray.Num(); i++) {
            const AMyPhysicsWorldActor::PhysicsObject& PhysicsObject = PhysicsObjectsArray[i];
            FVector x = PhysicsObject.Transform.GetLocation();
            x.Y *= -1;
            FRotator rot = PhysicsObject.Transform.Rotator();
            rot = PhysicsObject.Rotation;
            FVector RotEuler = rot.Euler();
            FVector Scale = PhysicsObject.Scale;
            FQuat quat = rot.Quaternion();
            // quat.Y = quat.Y;
            quat.X = -quat.X;
            quat.Z = -quat.Z;
            FString IDAssetName = AssetName + Scale.ToString();

            int subObjCount = PhysicsObject.SubObjectCount;
            if (!CreatedAssets.Contains(IDAssetName)) {

                // if(AssetName.Contains("_sub"))
                //     continue;

                if (PhysicsObject.RobotPart)
                    continue;

                // TODO: this is where we need to create the mesh assets in the XML
                if (subObjCount <= 1) {
                    AddMeshAsset(IDAssetName, PhysicsObject.ObjectPathOrPrimitive, Scale);
                    CreatedAssets.Add(IDAssetName);
                } else {
                    FString BaseFilePath = PhysicsObject.ObjectPathOrPrimitive;
                    FString BaseName = FPaths::GetBaseFilename(BaseFilePath);
                    FString Directory = FPaths::GetPath(BaseFilePath);

                    for (int32 j = 0; j < subObjCount; ++j) {
                        FString SubAssetName = IDAssetName + FString::Printf(TEXT("_sub_%d"), j);

                        FString SubFilePath = FString::Printf(TEXT("%s/%s_sub_%d.obj"), *Directory, *BaseName, j);
                        AddMeshAsset(SubAssetName, SubFilePath, Scale);
                        CreatedAssets.Add(SubAssetName);
                    }
                }
            } else {

                UE_LOG(LogTemp, Error, TEXT("%s already created "), *IDAssetName);
            }

            // RotEuler= PhysicsObject.Transform.GetRotation().Euler();

            // UE_LOG(LogTemp, Warning, TEXT("Creating with euler rotation of %s"), *RotEuler.ToString());
            RotEuler.X = FMath::DegreesToRadians(RotEuler.X);
            RotEuler.Y = FMath::DegreesToRadians(RotEuler.Y);
            RotEuler.Z = FMath::DegreesToRadians(RotEuler.Z);
            // INFO: The below is for static geom creation
            //  WorldBodyNode->AppendChildNode("geom", "",
            //                                 CreateMeshAttributesQuat(PhysicsObject.ObjectName, "mesh", "chairstl", x / 10.0f,
            //                                 quat));
            // INFO: The below is for dynamic body creation
            WorldBodyNode->AppendChildNode("body", "",
                                           CreateBodyAttributesQuat(PhysicsObject.ObjectName + "_body", x / 100.0f, quat, Scale));
            FXmlNode* LocalBodyNode = WorldBodyNode->FindChildNode("body");
            auto ChildNodes = WorldBodyNode->GetChildrenNodes();
            for (FXmlNode* n : ChildNodes) {
                if (n->GetAttribute("name").Equals(PhysicsObject.ObjectName + "_body")) {
                    if (!PhysicsObject.Static)
                        n->AppendChildNode("freejoint");

                    // TODO: this is where we need to add any of the child meshes from compplex if they exists
                    //  we can do this by checkign PhysicsObject.SUbOjbectCount
                    //  We will name them all with _mesh_n.obj if there is more than 1

                    if (subObjCount <= 1) {

                        n->AppendChildNode("geom", "",
                                           CreateMeshAttributesSimple(PhysicsObject.ObjectName, "mesh", "ASSET_" + IDAssetName));
                    } else {
                        FString BaseFilePath = PhysicsObject.ObjectPathOrPrimitive;
                        FString BaseName = FPaths::GetBaseFilename(BaseFilePath);
                        FString Directory = FPaths::GetPath(BaseFilePath);

                        for (int32 j = 0; j < subObjCount; ++j) {
                            FString SubAssetName = IDAssetName + FString::Printf(TEXT("_sub_%d"), j);
                            FString SubObjName = PhysicsObject.ObjectName + FString::Printf(TEXT("_sub_%d"), j);

                            n->AppendChildNode("geom", "", CreateMeshAttributesSimple(SubObjName, "mesh", "ASSET_" + SubAssetName));
                        }
                    }
                }
            }
        }
    }

    FString fp_save; //  = TEXT("C:\\Users\\jonem\\Downloads\\hello_mod.xml");

    // fp_save= FPaths::Combine(FPaths::ProjectContentDir(), TEXT("XMLDATA/hello_mod.xml"));

    fp = FString::Printf(TEXT("%shello_mod.xml"), *FPaths::ProjectSavedDir());
    fp_save = FPaths::ConvertRelativePathToFull(fp);
    m_File.Save(fp_save);
}
void XmlManager::CreateFreeNode() {}
void XmlManager::AddGeom() {}

FString XmlManager::GetFileAsString() {

    FString fp = TEXT("C:\\Users\\jonem\\Downloads\\hello.xml");
    m_File.LoadFile(fp);
    FString XmlContent;

    if (FFileHelper::LoadFileToString(XmlContent, *fp)) {
        UE_LOG(LogTemp, Error, TEXT("%s"), *XmlContent);
    } else {
        UE_LOG(LogTemp, Error, TEXT("Failed to load XML file."));
    }
    return XmlContent;
}

FXmlNode* XmlManager::FindWorldBodyNode() {
    if (FXmlNode* MujocoNode = m_File.GetRootNode()) {
        if (MujocoNode) {
            for (auto child : MujocoNode->GetChildrenNodes()) {
                if (child->GetTag().Equals("worldbody"))
                    return child;
            }
        }
        // return MujocoNode->FindChildNode(TEXT("worldbody"));
    }
    return nullptr;
}
