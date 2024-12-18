// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

using System;
using System.IO;
public
class Synthy : ModuleRules {
public
    Synthy(ReadOnlyTargetRules Target)
        : base(Target) {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicIncludePaths.AddRange(new string[]{
            // ... add public include paths required here ...
        });

        PrivateIncludePaths.AddRange(new string[]{
            // ... add other private include paths required here ...
        });

        PublicDependencyModuleNames.AddRange(new string[]{
            "Core", "XmlParser",
            // ... add other public dependencies that you statically link with here ...
        });

        PrivateDependencyModuleNames.AddRange(new string[]{
            "CoreUObject", "Engine", "Slate", "SlateCore", "CinematicCamera", "ImageWrapper",
            //"UnrealEd",
            "PhysicsCore", "Chaos", "XmlParser", "ROSIntegration", "CoACDInterface",  "Landscape"
            // ... add private dependencies that you statically link with here ...
        });

        DynamicallyLoadedModuleNames.AddRange(new string[]{
            // ... add any modules that your module loads dynamically here ...
        });

        // AddBullet();
        AddMuj();
        AddCoACD();
    }

private
    string ThirdPartyPath {
        get {
            return Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../ThirdParty/"));
        }
    }

protected
    void AddBullet() {
        // This is real basic, only for a single platform & config (Win64)
        // If you build for more variants you'll have to do some more work here

        bool bDebug =
            Target.Configuration == UnrealTargetConfiguration.Debug || Target.Configuration == UnrealTargetConfiguration.DebugGame;
        bool bDevelopment = Target.Configuration == UnrealTargetConfiguration.Development;

        string BuildFolder = bDebug ? "Debug" : bDevelopment ? "Debug" : "Release";
        string BuildSuffix = bDebug ? "_Debug" : bDevelopment ? "_Debug" : "";

        // Library path
        string LibrariesPath = Path.Combine(ThirdPartyPath, "lib", "bullet", BuildFolder);
        // PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "BulletCollision" + BuildSuffix + ".lib"));

        // PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "BulletDynamics" + BuildSuffix + ".lib"));
        // PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "LinearMath" + BuildSuffix + ".lib"));

        // string LibrariesPath = Path.Combine(ThirdPartyPath, "lib", "mujoco", BuildFolder);
        foreach (string LibFile in Directory.GetFiles(LibrariesPath, "*.lib")) {
            if (LibFile.Contains("Bullet2FileLoader"))
                continue;
            PublicAdditionalLibraries.Add(LibFile);
        }

        // Include path (I'm just using the source here since Bullet has mixed src & headers)
        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "bullet3", "src"));
        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "bullet3", "Extras", "BulletRobotics"));
        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "bullet3", "examples"));
        // PublicIncludePaths.Add( Path.Combine( ThirdPartyPath, "bullet3") );
        PublicDefinitions.Add("WITH_BULLET_BINDING=1");
    }

protected
    void AddMuj() {
        // This is real basic, only for a single platform & config (Win64)
        // If you build for more variants you'll have to do some more work here

        bool bDebug =
            Target.Configuration == UnrealTargetConfiguration.Debug || Target.Configuration == UnrealTargetConfiguration.DebugGame;
        bool bDevelopment = Target.Configuration == UnrealTargetConfiguration.Development;

        string BuildFolder = "Debug";
        //	bDevelopment ? "Debug" : "Release";
        string BuildSuffix = bDebug ? "_Debug" : bDevelopment ? "_Debug" : "";

        // Library path
        string LibrariesPath = Path.Combine(ThirdPartyPath, "lib", "mujoco", BuildFolder);
        foreach (string LibFile in Directory.GetFiles(LibrariesPath, "*.lib")) {
            PublicAdditionalLibraries.Add(LibFile);
        }

        // Include path (I'm just using the source here since Bullet has mixed src & headers)

        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "mujoco", "include"));
        PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "mujoco", "src"));

        string MujocoDLLPath = Path.Combine(ThirdPartyPath, "mujoco", "build", "bin", BuildFolder);
        PublicDelayLoadDLLs.Add("mujoco.dll"); // Delay load the DLL
        // Ensure the DLL is packaged
        RuntimeDependencies.Add(Path.Combine(MujocoDLLPath, "mujoco.dll"));

        // Add the DLL path to the environment
        PublicDefinitions.Add(string.Format("DLLIMPORTDIR=\"{0}\"", MujocoDLLPath.Replace("\\", "/")));

        // PublicDefinitions.Add("WITH_BULLET_BINDING=1");
        PublicDefinitions.Add("MUJOCO_INCLUDE_PREFIX=\"mujoco/\"");
    }

protected
    void AddCoACD() {

        string ThirdPartyPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../ThirdParty/"));
        string CoACDPath = Path.Combine(ThirdPartyPath, "CoACD");
        PublicIncludePaths.Add(Path.Combine(CoACDPath, "src"));
        PublicIncludePaths.Add(Path.Combine(CoACDPath, "public"));
        PublicIncludePaths.Add(Path.Combine(CoACDPath, "public/spdlog"));

        // Link libraries
        PublicAdditionalLibraries.Add(Path.Combine(CoACDPath, "build", "Release", "coacd.lib"));
        // PublicAdditionalLibraries.Add(Path.Combine(CoACDPath, "build", "Release", "_coacd.lib"));
        string libPath = Path.Combine(CoACDPath, "build", "Release");
        PublicDefinitions.Add("COACD_EXPORTS=1");

        foreach (string LibFile in Directory.GetFiles(libPath, "*.lib")) {
            PublicAdditionalLibraries.Add(LibFile);
        }

        PublicDelayLoadDLLs.Add(Path.Combine(CoACDPath, "build", "Release", "lib_coacd.dll"));

        RuntimeDependencies.Add(Path.Combine(CoACDPath, "build", "Release", "lib_coacd.dll"));
    }
}
