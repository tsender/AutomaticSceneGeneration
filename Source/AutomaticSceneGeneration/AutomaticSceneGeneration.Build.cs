// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System;
using System.IO;

public class AutomaticSceneGeneration : ModuleRules
{
	public AutomaticSceneGeneration(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		string rosintegrationPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../ROSIntegration/Source/ROSIntegration/Private"));
		
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
		
		PrivateIncludePaths.AddRange(new string[] {Path.Combine(ModuleDirectory, "Private"),
													rosintegrationPath, rosintegrationPath + "/rosbridge2cpp"});
			
		PublicDependencyModuleNames.AddRange(new string[] { "Core", 
															"CoreUObject", 
															"Engine", 
															"PhysXVehicles",
															/*"InputCore", 
															"RenderCore", 
															"RHI",*/
															"ROSIntegration" });
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"Json",
				"PhysXVehicles",
				"Slate",
				"SlateCore",
				"InputCore", 
				"RenderCore", 
				"RHI",
				"ROSIntegration",
				"Landscape",
				"ProceduralMeshComponent"
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
