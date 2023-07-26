#pragma once

#include <CoreMinimal.h>
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/geometry_msgs/Point.h"
#include "auto_scene_gen_msgs/msg/SceneDescription.h"
#include "auto_scene_gen_msgs/msg/SceneCaptureSettings.h"

namespace ROSMessages {
	namespace auto_scene_gen_msgs {
		// RunScenario requests are to be submitted by the ASG client. They describe the scenario that the ASG worker should run
		class FRunScenarioRequest : public FROSBaseServiceRequest {

		public:
			FRunScenarioRequest() = default;
			~FRunScenarioRequest() = default;

			// Mainly just used to check on progress
			int32 scenario_number;

			// Maximum amount of time [s] to let the simulation run before terminating. Set to -1 to disable feature.
			float sim_timeout_period;

			// Maximum amount of time [s] the vehicle can idle (once it began moving) before terminating the simulation. Set to -1 to disable feature.
			// Idling is defined as being at/near rest while also commanding zero velocity.
			float vehicle_idling_timeout_period;

			// Maximum amount of time [s] the vehicle can be "stuck", like on an obstacle, before terminating the simulation. Set to -t to disable feature.
			// We define the vehicle as being stuck if it is not moving, has not flipped over, but is still being sent non-zero control commands.
			float vehicle_stuck_timeout_period;

			// Max allowed vehicle roll angle [deg]. Simulation will end if this threshold is met.
			float max_vehicle_roll;

			// Max allowed vehicle pitch angle [deg]. Simulation will end if this threshold is met.
			float max_vehicle_pitch;

			// If true, then the simulator will not terminate the simulation if the vehicle touches a non-traversable obstacle.
			// If false, then the simulation will terminate with reason REASON_VEHICLE_COLLISION (see AnalyzeScenario.srv) if the vehicle touches a non-traversable obstacle.
			bool allow_collisions;
			
			// The Z location is ignored and will populated automatically by the ASG
			geometry_msgs::Point vehicle_start_location;
			
			float vehicle_start_yaw;
			
			// The Z location is ignored and will be populated automatically by the ASG
			geometry_msgs::Point vehicle_goal_location;

			// If vehicle is within this distance in [cm] of the goal location, then we assume the vehicle succeeded.
			float goal_radius;

			// The scene description
			auto_scene_gen_msgs::SceneDescription scene_description;

			// Indicates if we should take scene captures to send back to the client
			bool take_scene_capture;

			// Indicates if we should only take scene captures after creating the scene (the scneario will NOT be run).
			bool scene_capture_only;

			// Scene capture settings
			auto_scene_gen_msgs::SceneCaptureSettings scene_capture_settings;
		};
	}
}