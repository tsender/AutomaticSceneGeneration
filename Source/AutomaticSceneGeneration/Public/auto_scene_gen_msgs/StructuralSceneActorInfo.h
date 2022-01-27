#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		class StructuralSceneActorInfo: public FROSBaseMsg {
		public:
			StructuralSceneActorInfo()
            {
                _MessageType = "auto_scene_gen_msgs/StructuralSceneActorInfo";
			}

			// Path name for the SSA subclass
			FString path_name;

			// Number of SSAs we wish to place in the scene
			int32 count;

			// Concatenated attribute array for all of the SSAs
			ROSMessages::std_msgs::Float32MultiArray attr_array;
		};
	}
}
