#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/geometry_msgs/Point.h"
#include "auto_scene_gen_msgs/msg/StructuralSceneActorLayout.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// RunScenario requests are to be submitted by the ASG client. They describe the scenario that the ASG worker should run
		class FRunScenarioRequest : public FROSBaseServiceRequest {

		public:
			FRunScenarioRequest() = default;
			~FRunScenarioRequest() = default;

			// Indicates if we are done testing and the UE4 editor can end the game
			bool done_testing;

			// Mainly just used to check on progress
			int32 scenario_number;

			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_start_location;
			
			float vehicle_start_yaw;
			
			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_goal_location;

			// An array defining all of the types of SSAs and their attributes from which to place in the scene
			TArray<auto_scene_gen_msgs::StructuralSceneActorLayout> ssa_array;
		};
	}
}