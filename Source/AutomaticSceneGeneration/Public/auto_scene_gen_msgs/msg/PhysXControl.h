#pragma once

#include "ROSIntegration/Public/ROSBaseMsg.h"
#include "ROSIntegration/Public/std_msgs/Header.h"

namespace ROSMessages{
	namespace auto_scene_gen_msgs {
		// Message for sending basic PhysX control commands to a PhysX vehicle
		class PhysXControl: public FROSBaseMsg {
		public:
			PhysXControl() : PhysXControl(0, 0, true) {}

			PhysXControl(float longitudinal_velocity_in, float steering_angle_in, bool handbrake_in = true)
            {
				longitudinal_velocity = longitudinal_velocity_in;
                steering_angle = steering_angle_in;
                handbrake = handbrake_in;
                _MessageType = "auto_scene_gen_msgs/PhysXControl";
			}

			std_msgs::Header header;
			float longitudinal_velocity; // [cm/s]
			float steering_angle; // Range [-MaxSteeringAngle, +MaxSteeringAngle] [deg]
			bool handbrake; // false = disengaged, true = engaged
		};
	}
}
