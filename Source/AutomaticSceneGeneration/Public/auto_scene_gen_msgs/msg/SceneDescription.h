#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "auto_scene_gen_msgs/msg/StructuralSceneActorLayout.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This message contains information regarding the scene description
		class SceneDescription: public FROSBaseMsg {
		public:
			SceneDescription()
            {
                _MessageType = "auto_scene_gen_msgs/SceneDescription";
			}

			// The angle the sunlight makes with the horizontal [deg]
			float sunlight_inclination;

			// The yaw angle the sunlight is pointing in (i.e., the angle the shadow will be cast in) [deg]
			float sunlight_yaw_angle;
			
			// The mesh's scale (applies to all 3 axes)
			TArray<auto_scene_gen_msgs::StructuralSceneActorLayout> ssa_array;
		};
	}
}
