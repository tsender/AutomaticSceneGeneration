#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// This message defines the supported attributes that can be modified on structural scene actors (SSAs)
		class StructuralSceneAttributes: public FROSBaseMsg {
		public:
			StructuralSceneAttributes()
            {
                _MessageType = "auto_scene_gen_msgs/StructuralSceneAttributes";
			}

			// Indicates if the actor is visible
			bool visible;

			// X position in [cm]
			float x;

			// Y position in [cm]
			float y;

			// Yaw angle in [deg]
			float yaw;

			// Actor scale (applies to all 3 axes)
			float scale;
		};
	}
}
