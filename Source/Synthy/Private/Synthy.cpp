// Copyright Epic Games, Inc. All Rights Reserved.

#include "Synthy.h"
#include "Windows/WindowsPlatformProcess.h"
#include "Misc/Paths.h"
#include "Misc/MessageDialog.h"
#define LOCTEXT_NAMESPACE "FSynthyModule"

void FSynthyModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	FString MujocoDLLPath = FPaths::Combine(FPaths::ProjectDir(), TEXT("ThirdParty/mujoco/build/bin/Debug/mujoco.dll"));
	void* MujocoHandle = FPlatformProcess::GetDllHandle(*MujocoDLLPath);

	if (!MujocoHandle)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to load mujoco.dll from path: %s"), *MujocoDLLPath);
		return;  // Exit early or handle the error
	}
}

void FSynthyModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FSynthyModule, Synthy)
