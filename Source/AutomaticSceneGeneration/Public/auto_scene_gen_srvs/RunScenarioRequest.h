#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/geometry_msgs/Point.h"
#include "auto_scene_gen_msgs/StructuralSceneActorArray.h"

namespace ROSMessages {
	namespace auto_scene_gen_srvs {
		// The ASG client submits a RunScenario request describing the scenario that the ASG worker should run
		class FRunScenarioRequest : public FROSBaseServiceRequest {

		public:
			FRunScenarioRequest() = default;
			~FRunScenarioRequest() = default;

			bool done_testing;
			int32 scenario_number;

			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_start_location;
			
			float vehicle_start_yaw;
			
			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_goal_location;

			TArray<auto_scene_gen_msgs::StructuralSceneActorArray> ssa_array; // Structural scene actor array
		};
	}
}