// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutomaticSceneGeneration.h"

#define LOCTEXT_NAMESPACE "FAutomaticSceneGenerationModule"

void FAutomaticSceneGenerationModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FAutomaticSceneGenerationModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
// Apparently using IMPLEMENT_PRIMARY_GAME_MODULE allows for hot-reload to work. Switch back to IMPLEMENT_MODULE when done coding or if issues arise.
// IMPLEMENT_MODULE(FAutomaticSceneGenerationModule, AutomaticSceneGeneration)
IMPLEMENT_PRIMARY_GAME_MODULE( FDefaultGameModuleImpl, AutomaticSceneGeneration, "AutomaticSceneGeneration" )