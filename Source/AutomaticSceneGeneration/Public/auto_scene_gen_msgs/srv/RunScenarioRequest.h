#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/geometry_msgs/Point.h"
#include "auto_scene_gen_msgs/msg/SceneDescription.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// RunScenario requests are to be submitted by the ASG client. They describe the scenario that the ASG worker should run
		class FRunScenarioRequest : public FROSBaseServiceRequest {

		public:
			FRunScenarioRequest() = default;
			~FRunScenarioRequest() = default;

			// Mainly just used to check on progress
			int32 scenario_number;

			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_start_location;
			
			float vehicle_start_yaw;
			
			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_goal_location;

			// If vehicle is within this distance in [cm] of the goal location, then we assume the vehicle succeeded.
			float goal_radius;

			// The scene description
			auto_scene_gen_msgs::SceneDescription scene_description;
		};
	}
}