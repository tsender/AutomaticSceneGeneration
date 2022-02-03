#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This message specifies all of the attributes for the actors of a specific SSA subclass that we wish to place in a UE4 scene
		class StructuralSceneActorLayout: public FROSBaseMsg {
		public:
			StructuralSceneActorLayout()
            {
                _MessageType = "auto_scene_gen_msgs/StructuralSceneActorLayout";
			}

			// UE4 path name for the SSA subclass
			FString path_name;

			// For verifications purposes, this is used to determine the number of instances
			uint16 num_instances;

			// Indicates if the actor is visible in the game. Toggling this is more efficient than adding/removing elements from the arrays below
			TArray<bool> visibilities;

			// X positions in [cm]
			TArray<float> x_positions;

			// Y positions in [cm]
			TArray<float> y_positions;

			// Yaw angles in [deg]
			TArray<float> yaw_angles;

			// The mesh's scale (applies to all 3 axes)
			TArray<float> scale_factors;
		};
	}
}
