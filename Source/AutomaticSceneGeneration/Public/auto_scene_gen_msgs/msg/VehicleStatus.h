#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		class VehicleStatus: public FROSBaseMsg {
		public:
			VehicleStatus() : VehicleStatus(false, false) {}

			VehicleStatus(bool InEnable, bool InPreempted)
            {
				enabled = InEnable;
				preempted = InPreempted;
                _MessageType = "auto_scene_gen_msgs/VehicleStatus";
			}

			// Indicates if the vehicle is enabled
			bool enabled;

			// Indicates if the vehicle was disabled preemptively (only applies if enabled is False)
			bool preempted;
		};
	}
}
