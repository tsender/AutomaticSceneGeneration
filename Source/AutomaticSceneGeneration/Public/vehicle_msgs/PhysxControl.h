#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"

namespace ROSMessages{
	namespace vehicle_msgs {
		class PhysXControl: public FROSBaseMsg {
		public:
			PhysXControl() : PhysXControl(0, 0, false) {}

			PhysXControl(float longitudinal_velocity_in, float steering_angle_in, bool handbrake_in = false)
            {
				longitudinal_velocity = longitudinal_velocity_in;
                steering_angle = steering_angle_in;
                handbrake = handbrake_in;
                _MessageType = "vehicle_msgs/PhysXControl";
			}

			float longitudinal_velocity; // [m/s]
			float steering_angle; // Range [-MaxSteeringAngle, +MaxSteeringAngle] [deg]
			bool handbrake; // false = disengaged, true = engaged
		};
	}
}
