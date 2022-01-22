#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace vehicle_msgs {
		class PhysxControl: public FROSBaseMsg {
		public:
			PhysxControl() : PhysxControl(0, 0, false) {}

			PhysxControl(float longitudinal_velocity_in, float steering_angle_in, bool handbrake_in = false)
            {
				longitudinal_velocity = longitudinal_velocity_in;
                steering_angle = steering_angle_in;
                handbrake = handbrake_in;
                _MessageType = "vehicle_msgs/PhysxControl";
			}

			float longitudinal_velocity; // [m/s]
			float steering_angle; // In range +/- MaxSteeringAngle [deg]
			bool handbrake; // false = disengaged, true = engaged
		};
	}
}
