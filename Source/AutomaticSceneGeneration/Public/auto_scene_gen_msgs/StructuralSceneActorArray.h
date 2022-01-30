#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "auto_scene_gen_msgs/StructuralSceneAttributes.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This message specifies all of the attributes for the actors of a specific SSA subclass that we wish to place in a UE4 scene
		class StructuralSceneActorArray: public FROSBaseMsg {
		public:
			StructuralSceneActorArray()
            {
                _MessageType = "auto_scene_gen_msgs/StructuralSceneActorArray";
			}

			// UE4 path name for the SSA subclass
			FString path_name;

			// Array of the structural scene attributes for the SSAs
			TArray<auto_scene_gen_msgs::StructuralSceneAttributes> attr_array;
		};
	}
}
