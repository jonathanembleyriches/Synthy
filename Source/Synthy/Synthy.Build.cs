// Copyright Epic Games, Inc. All Rights Reserved.

using System;
using UnrealBuildTool;
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
            "Core", "XmlParser", "Engine",
            // ... add other public dependencies that you statically link with here ...
        });

        PrivateDependencyModuleNames.AddRange(new string[]{
            "CoreUObject", "Engine", "Slate", "SlateCore", "CinematicCamera", "ImageWrapper",
            //"UnrealEd",
            "PhysicsCore", "Chaos", "XmlParser", "ROSIntegration", "CoACDInterface", "Landscape", "RHI", "RenderCore"
            // ... add private dependencies that you statically link with here ...
        });

        DynamicallyLoadedModuleNames.AddRange(new string[]{
            // ... add any modules that your module loads dynamically here ...
        });

        // AddBullet();
        AddMuj();
        AddCoACD();
        AddZeroMQ();
    }

private
    string ThirdPartyPath {
        get {
            return Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../ThirdParty/"));
        }
    }

protected
    void AddZeroMQ() {
        // Base paths
        string PluginBasePath = ModuleDirectory;
        // string ThirdPartyPath = Path.GetFullPath(Path.Combine(PluginBasePath, "..", "..", "ThirdParty", "zmq"));

        // Include, library, and binary paths
        string ZmqIncludePath = Path.Combine(ThirdPartyPath, "zmq", "include");
        string ZmqLibPath = Path.Combine(ThirdPartyPath, "zmq", "lib");
        string ZmqBinPath = Path.Combine(ThirdPartyPath, "zmq", "bin");

        foreach (string LibFile in Directory.GetFiles(ZmqLibPath, "*.lib")) {
            PublicAdditionalLibraries.Add(LibFile);
        }

        // Include path (I'm just using the source here since Bullet has mixed src & headers)

        PublicIncludePaths.Add(ZmqIncludePath);

        PublicDelayLoadDLLs.Add("libzmq-mt-4_3_5.dll"); // Delay load the DLL
        // Ensure the DLL is packaged
        RuntimeDependencies.Add(Path.Combine(ZmqBinPath, "libzmq-mt-4_3_5.dll"));

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
